#!/usr/bin/env python3
"""Estudo exploratório ATSC v3.0 – Curitiba (3+3 e 3 faixas). Python 3.10+; SUMO >= 1.14."""
import argparse, csv, json, math, os, random, shutil, statistics, subprocess, sys
from pathlib import Path
import xml.etree.ElementTree as ET
from collections import Counter

ROOT = Path(__file__).resolve().parent
CYCLE, AVAILABLE, MIN_GREEN, MAX_GREEN = 89, 77, 20, 57
HIST = 45
FLOWS = {'D':1720, 'E':1920, 'L':1400}
SAT = {'D':3846.35, 'E':4190.65, 'L':4085.33}
GROUPS = {g:tuple(range(3)) for g in FLOWS}
IN_EDGES = {'D':'direita_in','E':'esquerda_in','L':'side_in'}
OUT_EDGES = {'D':'direita_out','E':'esquerda_out','L':'side_out'}
LANE_WIDTH = 3.2
MEDIAN = 2.0  # distância ilustrativa entre bordas das pistas, não medida

PROFILES = {'constante':[(1,1)]*4,'variavel':[(1.25,.75),(1.25,.75),(.75,1.25),(.75,1.25)]}
def normalized(a,b):
    assert len(a)==len(b) and min(a+b)>0
    return [(x/statistics.mean(a),y/statistics.mean(b)) for x,y in zip(a,b)]

# Equal-duration bins, each axis mean factor = 1. Original profiles unchanged.
PROFILES.update({
    'ondas':normalized([.8,1.2,1.4,.9,.65,1.15,1.35,.75,1.2,.85,1.05,.7], [1.2,.8,.6,1.1,1.35,.85,.65,1.25,.8,1.15,.95,1.3]),
    'pico':normalized([.65,.75,.9,1.1,1.3,1.5,1.4,1.2,1,.85,.7,.65], [.7,.75,.8,.85,.95,1,1.15,1.3,1.45,1.3,1,.75]),
    'pelotoes':[(1.7,.3),(.3,1.7)]*60,
})
# OECD 2020 table 3.7, conventional passenger car, g PM10/km; not local measurements.
PM10_FACTORS={'freios':.0074,'pneus':.0064,'pavimento':.0075,'poeira':.0083}
MASS_KG=1500.0 # hypothetical; proxy only
CANDIDATES = [35,39,43,45,47,51,55]
SCENARIOS = {'A':'Fixo histórico','B':'Fixo redimensionado','C':'Adaptativo original','D':'Adaptativo aprimorado'}

def binary(name):
    # Works for system installation, official SUMO_HOME or eclipse-sumo pip wheel.
    dirs = []
    if os.environ.get('SUMO_HOME'): dirs.append(Path(os.environ['SUMO_HOME'])/'bin')
    try:
        import sumo
        dirs.append(Path(sumo.__file__).resolve().parent/'bin')
    except (ImportError, TypeError): pass
    dirs.append(Path(sys.executable).parent/'Scripts')
    for d in dirs:
        for suffix in ('','.exe'):
            p=d/(name+suffix)
            if p.is_file(): return str(p)
    found=shutil.which(name)
    if found: return found
    try:
        import sumolib
        p=sumolib.checkBinary(name)
        if Path(p).is_file() or shutil.which(p): return p
    except ImportError: pass
    raise RuntimeError(f'{name} não encontrado. Confira SUMO_HOME e a instalação eclipse-sumo.')

def write_xml(path, root):
    ET.indent(root, space='  ')
    ET.ElementTree(root).write(path, encoding='utf-8', xml_declaration=True)

def network_trees():
    """Two parallel one-way carriageways; one common conflict zone and signal.
    Shapes are illustrative. No cross-median lane changes, no turns.
    """
    nodes=ET.Element('nodes'); edges=ET.Element('edges'); conns=ET.Element('connections')
    offset=(3*LANE_WIDTH+MEDIAN)/2
    # Westbound right carriageway is north of left carriageway.
    for group,y in [('D',offset),('E',-offset)]:
        for suffix,x in [('src',300),('dst',-150)]:
            ET.SubElement(nodes,'node',id=group+'_'+suffix,x=str(x),y=str(y),type='priority')
    for id,x,y in [('L_src',0,-300),('L_dst',0,150)]:
        ET.SubElement(nodes,'node',id=id,x=str(x),y=str(y),type='priority')
    ET.SubElement(nodes,'node',id='J',x='0',y='0',type='traffic_light',shape='-8,-13 8,-13 8,13 -8,13')
    for g in FLOWS:
        y=offset if g=='D' else -offset
        shapes=(f'300,{y} 8,{y}',f'-8,{y} -150,{y}') if g!='L' else ('0,-300 0,-13','0,13 0,150')
        for edge,fr,to,shape in [(IN_EDGES[g],g+'_src','J',shapes[0]),(OUT_EDGES[g],'J',g+'_dst',shapes[1])]:
            ET.SubElement(edges,'edge',{'id':edge,'from':fr,'to':to,'numLanes':'3','speed':str(50/3.6),'width':str(LANE_WIDTH),'priority':'1' if g=='L' else '2','spreadType':'center','shape':shape})
        for lane in GROUPS[g]:
            ET.SubElement(conns,'connection',{'from':IN_EDGES[g],'to':OUT_EDGES[g],'fromLane':str(lane),'toLane':str(lane)})
    return nodes,edges,conns

def validate_network(net):
    """Fail early if netconvert changes lanes, topology or controlled links."""
    root=ET.parse(net).getroot()
    edges={e.attrib['id']:e for e in root.findall('edge') if not e.attrib['id'].startswith(':')}
    assert set(edges)==set(IN_EDGES.values())|set(OUT_EDGES.values()), 'arestas inesperadas'
    for e in edges.values(): assert len(e.findall('lane'))==3, 'quantidade de faixas incorreta'
    expected={(IN_EDGES[g],OUT_EDGES[g],str(i),str(i)) for g in FLOWS for i in GROUPS[g]}
    actual={(c.get('from'),c.get('to'),c.get('fromLane'),c.get('toLane')) for c in root.findall('connection') if c.get('from') in IN_EDGES.values()}
    assert actual==expected, f'conexões inesperadas: {actual ^ expected}'
    controlled=[c for c in root.findall('connection') if c.get('from') in IN_EDGES.values()]
    assert all(c.get('tl')=='J' and c.get('linkIndex') is not None for c in controlled), 'movimento não semaforizado'
    # Check median at the upstream half (geometric separation, not just a visual line).
    ys={g:[float(l.get('shape').split()[0].split(',')[1]) for l in edges[IN_EDGES[g]].findall('lane')] for g in ('D','E')}
    assert min(ys['D'])-max(ys['E'])>LANE_WIDTH, 'pistas sobrepostas'
    return {'faixas':{g:3 for g in FLOWS},'movimentos':'somente em frente','conexoes_controladas':len(controlled),'largura_faixa_m':LANE_WIDTH,'mureta_m':MEDIAN,'geometria_metrica':'hipotetica'}

def build_network(folder):
    folder.mkdir(parents=True, exist_ok=True)
    nodes,edges,conns=network_trees()
    for name,tree in [('rede.nod.xml',nodes),('rede.edg.xml',edges),('rede.con.xml',conns)]: write_xml(folder/name,tree)
    net=folder/'rede.net.xml'
    cmd=[binary('netconvert'),'--node-files',str(folder/'rede.nod.xml'),'--edge-files',str(folder/'rede.edg.xml'),'--connection-files',str(folder/'rede.con.xml'),'--output-file',str(net)]
    res=subprocess.run(cmd,capture_output=True,text=True,encoding='utf-8',errors='replace')
    (folder/'netconvert.log').write_text(res.stdout+'\n'+res.stderr,encoding='utf-8')
    if res.returncode: raise RuntimeError('netconvert falhou: '+res.stderr)
    geometry=validate_network(net)
    (folder/'geometria.json').write_text(json.dumps(geometry,indent=2,ensure_ascii=False),encoding='utf-8')
    return net

def capacity_reference():
    # Published estimated saturation by carriageway; diagnostic, not calibration.
    return {g:{'fluxo_vpa_h':FLOWS[g],'saturacao_publicada_vpa_h':SAT[g],
               'capacidade_nominal_vpa_h':SAT[g]*(32 if g=='L' else 45)/CYCLE,
               'grau_saturacao_nominal':FLOWS[g]/(SAT[g]*(32 if g=='L' else 45)/CYCLE)} for g in FLOWS}

def demand(seed, profile, duration, scale=1., turns=0.):
    """Poisson arrivals by segment, exact same file across strategies."""
    if turns: raise ValueError("Conversões não modeladas na v3: faltam dados por movimento.")
    events=[]
    for gi,(group,flow) in enumerate(FLOWS.items()):
        rng=random.Random(seed*1009+gi*7919); rng2=random.Random(seed*2039+gi*389)
        for j,(fm,fs) in enumerate(PROFILES[profile]):
            start=duration*j/len(PROFILES[profile]); end=duration*(j+1)/len(PROFILES[profile])
            rate=flow*scale*(fs if group=='L' else fm)/3600
            t=start+rng.expovariate(rate)
            while t<end:
                # Preserve RNG draw from v2 for D/L (same arrival streams; lane assignment changes).
                if group in ('D','L'): rng2.random()
                lane=rng2.choice(GROUPS[group])
                route=IN_EDGES[group]+' '+OUT_EDGES[group]
                events.append((round(t,3),group,lane,route))
                t+=rng.expovariate(rate)
    events.sort()
    # rounding can land at duration, clamp below cutoff
    return [(min(t,duration-.001),f'{g}_{i:06d}',g,lane,route) for i,(t,g,lane,route) in enumerate(events)]

def write_routes(path, events, emission):
    root=ET.Element('routes')
    ET.SubElement(root,'vType',id='equivalente',vClass='passenger',carFollowModel='Krauss',length='4.5',minGap='2.5',tau='1.0',accel='2.6',decel='4.5',sigma='0.5',speedFactor='1.0',speedDev='0.0',emissionClass=emission)
    for t,id,g,lane,route in events:
        v=ET.SubElement(root,'vehicle',id=id,type='equivalente',depart=f'{t:.3f}',departLane=str(lane),departSpeed='max')
        ET.SubElement(v,'route',edges=route)
    write_xml(path,root)

def phase_at(t,green):
    k=t%CYCLE
    if k<green:return 'M','G'
    if k<green+4:return 'M','y'
    if k<green+6:return '-','r'
    if k<green+6+AVAILABLE-green:return 'L','G'
    if k<CYCLE-2:return 'L','y'
    return '-','r'

def adaptive_green(last_green, counts, queues):
    # Load = observed arrivals in last cycle + a queue-clearance term, normalized by saturation.
    m=max((counts[g]*3600/CYCLE+queues[g]*3600/CYCLE)/SAT[g] for g in ('D','E'))
    l=(counts['L']+queues['L'])*3600/CYCLE/SAT['L']
    target=round(AVAILABLE*m/(m+l)) if m+l else HIST
    target=max(MIN_GREEN,min(MAX_GREEN,target))
    return max(last_green-8,min(last_green+8,target))

def improved_green(last_green, anchor, smoothed, residual):
    # Feed-forward anchored to independently trained B; normalized to nominal demand.
    # Uses previous-cycle offered arrivals (including blocked vehicles), not future events.
    m=max(smoothed[g]/FLOWS[g] for g in ('D','E'))
    l=smoothed['L']/FLOWS['L']
    denom=anchor*m+(AVAILABLE-anchor)*l
    target=AVAILABLE*anchor*m/denom if denom>0 else anchor
    # End-of-own-green residual queue, including boundary backlog, in seconds of service.
    # Compare like phases rather than raw queues sampled at the end of the cycle.
    rm=max(residual[g]*3600/SAT[g] for g in ('D','E'))
    rl=residual['L']*3600/SAT['L']
    target+=max(-10,min(10,.35*(rm-rl)))
    target=max(MIN_GREEN,min(MAX_GREEN,round(target)))
    return max(MIN_GREEN,min(MAX_GREEN,max(last_green-5,min(last_green+5,target))))

def environmental_extras(distance, kinetic_loss_j, hard_steps, n):
    out={f'pm10_{k}_g':distance*v for k,v in PM10_FACTORS.items()}
    out['pm10_nao_escape_g']=sum(out.values())
    out['energia_desaceleracao_MJ']=kinetic_loss_j/1e6
    out['desaceleracoes_fortes_s_veic']=hard_steps/n if n else None
    return out

def signal_groups(links):
    groups=[]
    for ll in links:
        incoming={a[0].rsplit('_',1)[0] for a in ll}
        if incoming and incoming <= {IN_EDGES['D'],IN_EDGES['E']}: groups.append('M')
        elif incoming=={IN_EDGES['L']}: groups.append('L')
        else: raise RuntimeError('Link semafórico inesperado: '+repr(ll))
    if set(groups)!={'M','L'}: raise RuntimeError('Rede sem os dois grupos de sinais')
    return groups

def queue_by_group(c):
    return {g:sum(c.lane.getLastStepHaltingNumber(IN_EDGES[g]+'_'+str(i)) for i in lanes) for g,lanes in GROUPS.items()}

def parse_tripinfo(path, events):
    # SUMO >=1.14: fuel_abs and all pollutant masses in mg.
    lookup={v[1]:v[0] for v in events}; done=[]; masses=Counter(); missing_emissions=0; seen=set(); unfinished=0
    for _, el in ET.iterparse(path,events=('end',)):
        if el.tag!='tripinfo':continue
        id=el.attrib['id']; seen.add(id); a=el.attrib
        e=el.find('emissions')
        if e is None:missing_emissions+=1
        else:
            for key in ('CO2_abs','fuel_abs','NOx_abs','PMx_abs','CO_abs','HC_abs'):
                if key not in e.attrib: raise RuntimeError('Campo de emissão ausente: '+key)
                masses[key]+=float(e.attrib[key])
        if float(a['arrival'])>=0:
            # Use actual departure time minus scheduled departure: includes boundary queue.
            entry=max(0,float(a['depart'])-lookup[id]); loss=float(a['timeLoss']);
            done.append((id, entry, loss, float(a['duration'])+entry,float(a['waitingTime']),int(a['waitingCount']),float(a['routeLength'])))
        else:unfinished+=1
        el.clear()
    if missing_emissions or not seen: raise RuntimeError('Saída de emissões incompleta ou sem viagens: '+str(path))
    n=len(done); mean=lambda idx: statistics.mean(d[idx] for d in done) if n else None
    by={}
    for g in FLOWS:
        rows=[d for d in done if d[0].startswith(g+'_')]
        by[g]=statistics.mean(d[1]+d[2] for d in rows) if rows else None
    return dict(concluidos=n,nao_concluidos=len(events)-n,nao_inseridos=len(events)-len(seen),atraso_medio_s=statistics.mean(d[1]+d[2] for d in done) if n else None,espera_entrada_s=mean(1),viagem_media_s=mean(3),espera_parado_s=mean(4),paradas_media=mean(5),distancia_km=sum(d[6] for d in done)/1000,velocidade_media_kmh=(sum(d[6] for d in done)/sum(d[3]-d[1] for d in done)*3.6 if n and sum(d[3]-d[1] for d in done)>0 else None),co2_kg=masses['CO2_abs']/1e6,combustivel_kg=masses['fuel_abs']/1e6,nox_g=masses['NOx_abs']/1000,pm_g=masses['PMx_abs']/1000,co_g=masses['CO_abs']/1000,hc_g=masses['HC_abs']/1000,atraso_D_s=by['D'],atraso_E_s=by['E'],atraso_L_s=by['L'])

def run(net, routes, events, args, run_dir, scenario, green, seed, profile, gui=False, trace=False):
    import traci
    from traci import constants as tc
    run_dir.mkdir(parents=True,exist_ok=True); trip=run_dir/'tripinfo.xml'
    cmd=[binary('sumo-gui' if gui else 'sumo'),'-n',str(net),'-r',str(routes),'--seed',str(seed),'--step-length','1','--route-steps','0','--time-to-teleport','-1','--max-depart-delay','-1','--collision.action','warn','--tripinfo-output',str(trip),'--tripinfo-output.write-unfinished','true','--device.emissions.probability','1','--no-step-log','true','--duration-log.disable','true','--log',str(run_dir/'sumo.log')]
    if gui:cmd+=['--start','--quit-on-end','--delay','20']
    c=None; timeline=[]; plans=[]; qmax=0; qmmax=0; qlmax=0; qsum=0; ext_sum=0; extmax=0; arrivals=Counter(); inserted=0; completed=0; teleports=0; collisions=0; idx=0; horizon_done=None; previous=None
    offered=Counter(); waiting_ext=Counter(); residual=Counter(); smooth=dict(FLOWS)
    old_speed={}; kinetic_loss=0.; hard_steps=0
    anchor=green
    try:
        traci.start(cmd); c=traci.getConnection('default'); groups=signal_groups(c.trafficlight.getControlledLinks('J')); g=green
        for t in range(args.duration+args.drain):
            if t%CYCLE==0:
                if scenario=='C' and t: g=adaptive_green(g,arrivals,queue_by_group(c))
                if scenario=='D' and t:
                    for group in FLOWS:
                        smooth[group]=.65*offered[group]*3600/CYCLE+.35*smooth[group]
                    g=improved_green(g,anchor,smooth,residual)
                arrivals=Counter(); offered=Counter(); plans.append({'t':t,'verde_visconde':g,'verde_laurindo':AVAILABLE-g})
            phase,colour=phase_at(t,g); state=''.join(colour if x==phase else 'r' for x in groups)
            if state!=previous: c.trafficlight.setRedYellowGreenState('J',state);previous=state
            c.simulationStep()
            dep=c.simulation.getDepartedIDList(); inserted+=len(dep); arrivals.update(id.split('_')[0] for id in dep)
            completed+=c.simulation.getArrivedNumber(); teleports+=c.simulation.getStartingTeleportNumber(); collisions+=c.simulation.getCollidingVehiclesNumber()
            now=t+1
            while idx<len(events) and events[idx][0]<now:
                group=events[idx][2]; offered[group]+=1; waiting_ext[group]+=1; idx+=1
            for vid in dep:
                waiting_ext[vid.split('_')[0]]-=1
            # Speed subscription is observational only; no changes to vehicle behaviour.
            active=c.vehicle.getAllSubscriptionResults()
            new_speed={}
            for vid,values in active.items():
                v=values.get(tc.VAR_SPEED)
                if v is None or v<0:continue
                new_speed[vid]=v
                if vid in old_speed:
                    prev=old_speed[vid]
                    if prev-v>=.1: kinetic_loss+=.5*MASS_KG*(prev*prev-v*v)
                    if prev-v>=2: hard_steps+=1
            # Newly inserted vehicles are subscribed for following steps. Avoid subscribing
            # vehicles that have already arrived (not expected on this network).
            arrived_ids=set(c.simulation.getArrivedIDList())
            for vid in dep:
                if vid not in arrived_ids:
                    c.vehicle.subscribe(vid,(tc.VAR_SPEED,))
                    v=c.vehicle.getSubscriptionResults(vid).get(tc.VAR_SPEED)
                    if v is not None and v>=0:new_speed[vid]=v
            old_speed=new_speed
            external=max(0,idx-inserted)
            # queues are sampled each second for transparent comparable metrics.
            qs=queue_by_group(c)
            # End of each own green: residual queue before yellow starts.
            if t%CYCLE==g-1:
                for group in ('D','E'):residual[group]=qs[group]+max(0,waiting_ext[group])
            if t%CYCLE==CYCLE-7: # last second of Laurindo green (index 82)
                residual['L']=qs['L']+max(0,waiting_ext['L'])
            qm=qs['D']+qs['E']; ql=qs['L']; qtotal=qm+ql+external
            if now<=args.duration:
                qmax=max(qmax,qtotal);qmmax=max(qmmax,qm);qlmax=max(qlmax,ql);qsum+=qtotal;ext_sum+=external;extmax=max(extmax,external)
            if now==args.duration: horizon_done=completed
            if trace and (now%5==0 or now==args.duration):timeline.append({'t':now,'fila_visconde':qm,'fila_laurindo':ql,'fila_externa':external,'concluidos':completed,'verde_visconde':g,'fase':phase+colour})
            if now>=args.duration and completed==len(events):break
    finally:
        if c is not None:c.close()
    stats=parse_tripinfo(trip,events)
    if completed!=stats['concluidos']:raise RuntimeError('Contagem de chegadas diverge de tripinfo')
    if inserted>len(events) or completed>inserted:raise RuntimeError('Conservação de demanda violada')
    valid=(completed==len(events) and teleports==0 and collisions==0 and inserted==len(events))
    row=dict(cenario=scenario,perfil=profile,semente=seed,oferecidos=len(events),inseridos=inserted,concluidos_horizonte=horizon_done,fila_max_veic=qmax,fila_media_veic=qsum/args.duration,fila_visconde_max=qmmax,fila_laurindo_max=qlmax,fila_externa_max=extmax,fila_externa_media=ext_sum/args.duration,tempo_total_s=now,verde_visconde_medio_s=statistics.mean(p['verde_visconde'] for p in plans if p['t']<args.duration),teleportes=teleports,colisoes_detectadas=collisions,valido=valid,**stats,**environmental_extras(stats['distancia_km'],kinetic_loss,hard_steps,stats['concluidos']))
    if trace: (run_dir/'serie.json').write_text(json.dumps({'fila':timeline,'ciclos':plans},ensure_ascii=False),encoding='utf-8')
    (run_dir/'metricas.json').write_text(json.dumps(row,indent=2,ensure_ascii=False),encoding='utf-8')
    delay = row['atraso_medio_s']
    print(f"  {scenario} {profile} semente={seed}: {completed}/{len(events)} concluídos; atraso={delay if delay is not None else 'n/d'} s; CO2={row['co2_kg']:.2f}kg; {'OK' if valid else 'INCOMPLETO / INVÁLIDO'}",flush=True)
    return row

def write_csv(path,rows):
    if not rows:return
    with path.open('w',encoding='utf-8-sig',newline='') as f:
        w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)

def aggregate(rows):
    result=[]
    for profile in PROFILES:
        for s in SCENARIOS:
            group=[r for r in rows if r['perfil']==profile and r['cenario']==s]
            if not group:continue
            valid=[r for r in group if r['valido']]
            # Do not silently omit failed experiments: mark group incomplete, withhold comparisons.
            out={'perfil':profile,'cenario':s,'n':len(group),'n_validos':len(valid),'comparavel':len(valid)==len(group)}
            for key in ('oferecidos','concluidos_horizonte','atraso_medio_s','viagem_media_s','espera_parado_s','espera_entrada_s','paradas_media','fila_max_veic','fila_media_veic','fila_externa_max','atraso_D_s','atraso_E_s','atraso_L_s','velocidade_media_kmh','distancia_km','co2_kg','combustivel_kg','nox_g','pm_g','verde_visconde_medio_s','co_g','hc_g','energia_desaceleracao_MJ','desaceleracoes_fortes_s_veic','pm10_freios_g','pm10_pneus_g','pm10_pavimento_g','pm10_poeira_g','pm10_nao_escape_g'):
                vals=[r[key] for r in valid if r[key] is not None]
                out[key]=statistics.mean(vals) if len(vals)==len(group) else None
                if key in ('atraso_medio_s','co2_kg','combustivel_kg'):out[key+'_dp']=statistics.stdev(vals) if len(vals)==len(group) and len(vals)>1 else None
            result.append(out)
    return result

def self_test():
    for g in range(MIN_GREEN,MAX_GREEN+1):
        c=Counter(phase_at(t,g) for t in range(CYCLE));assert c['M','G']==g and c['L','G']==AVAILABLE-g and c['M','y']==4 and c['L','y']==4 and c['-','r']==4
    for profile in PROFILES:
        a=demand(42,profile,3600);assert a==demand(42,profile,3600);assert len({x[1] for x in a})==len(a);assert all(0<=x[0]<3600 for x in a)
        assert 4500<len(a)<5500
    assert adaptive_green(45,Counter(D=100,E=100,L=0),Counter())==53
    assert adaptive_green(45,Counter(L=100),Counter())==37
    assert adaptive_green(45,Counter(),Counter())==45
    for profile,bins in PROFILES.items():
        assert all(abs(statistics.mean(v)-1)<1e-10 for v in zip(*bins))
    for g in range(MIN_GREEN,MAX_GREEN+1):
        assert improved_green(g,43,FLOWS,Counter())==max(g-5,min(g+5,43))
        assert MIN_GREEN<=improved_green(g,43,dict(D=10000,E=10000,L=0),Counter())<=MAX_GREEN
        assert MIN_GREEN<=improved_green(g,43,dict(D=0,E=0,L=10000),Counter())<=MAX_GREEN
    assert improved_green(43,43,FLOWS,Counter(L=100))<43
    assert improved_green(43,43,FLOWS,Counter(D=100))>43
    assert abs(environmental_extras(1000,1e6,10,5)['pm10_nao_escape_g']-29.6)<1e-9
    assert signal_groups([[('direita_in_0','direita_out_0','x')],[('esquerda_in_2','esquerda_out_2','y')],[('side_in_0','side_out_0','z')]])==['M','M','L']
    nodes,edges,conns=network_trees();assert len(edges)==6 and len(conns)==9
    for profile in PROFILES:
        assert all(lane in GROUPS[group] and route==IN_EDGES[group]+' '+OUT_EDGES[group] for _,_,group,lane,route in demand(42,profile,3600))
    assert all(v['grau_saturacao_nominal']>0 for v in capacity_reference().values())
    print('OK: fases, restrições, controlador, demanda determinística e grupos semafóricos.')

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--rapido',action='store_true',help='1 semente por perfil e grade menor de treinamento')
    p.add_argument('--gui',action='store_true',help='visualiza uma execução única, sem treinamento')
    p.add_argument('--cenario',choices=SCENARIOS,default='C');p.add_argument('--perfil',choices=PROFILES,default='variavel')
    p.add_argument('--perfis',nargs='+',choices=PROFILES,default=list(PROFILES),help='Subconjunto de perfis para os experimentos');p.add_argument('--seeds',type=int,nargs='+',default=[42,43,44]);p.add_argument('--duration',type=int,default=3600);p.add_argument('--drain',type=int,default=3600)
    p.add_argument('--scale',type=float,default=1.0);p.add_argument('--turns',type=float,default=0.0,help='Reservado à compatibilidade: v3 aceita apenas 0 (sem dados de conversões)')
    p.add_argument('--emission-class',default='HBEFA3/PC_G_EU4');p.add_argument('--verde-fixo',type=int,default=43,help='Verde de B apenas no modo GUI')
    p.add_argument('--out',type=Path,default=ROOT/'resultados');p.add_argument('--self-test',action='store_true');args=p.parse_args()
    self_test()
    if args.self_test:return
    if args.duration<360 or args.drain<60 or not 0<args.scale<=2 or args.turns!=0 or not MIN_GREEN<=args.verde_fixo<=MAX_GREEN:p.error('Parâmetros fora dos limites')
    if len(set(args.seeds))!=len(args.seeds) or any(s<0 or s>2147483647 for s in args.seeds):p.error('Use sementes distintas, inteiras não negativas')
    if 10001 in args.seeds:p.error('Semente 10001 reservada ao treinamento')
    args.out=args.out.resolve();args.out.mkdir(parents=True,exist_ok=True)
    # Do not overwrite a completed dataset or leave stale report from prior run.
    if (args.out/'resumo.csv').exists() and not args.gui:raise RuntimeError('Pasta já contém resultados. Escolha outra com --out.')
    import traci
    version=subprocess.check_output([binary('sumo'),'--version'],text=True,encoding='utf-8',errors='replace').splitlines()[0]
    import re
    m=re.search(r'(\d+)\.(\d+)',version)
    if not m or tuple(map(int,m.groups()))<(1,14):raise RuntimeError('Requer SUMO 1.14+ para unidades de combustível em mg: '+version)
    print(f'OK: SUMO {m.group(0)} identificado corretamente.')
    net=build_network(args.out/'rede')
    (args.out/'capacidade_referencia.json').write_text(json.dumps(capacity_reference(),indent=2,ensure_ascii=False),encoding='utf-8')
    if args.gui:
        seed=args.seeds[0];events=demand(seed,args.perfil,args.duration,args.scale,args.turns);rdir=args.out/'gui';rdir.mkdir(exist_ok=True);route=rdir/'demanda.rou.xml';write_routes(route,events,args.emission_class)
        run(net,route,events,args,rdir,args.cenario,args.verde_fixo if args.cenario in ('B','D') else HIST,seed,args.perfil,True,True);return
    seeds=args.seeds[:1] if args.rapido else args.seeds
    # independent training, nominal demand; A is included among candidates.
    print('Treinamento independente do plano fixo B (semente 10001, perfil constante)...',flush=True)
    train_dir=args.out/'treinamento';train_dir.mkdir(exist_ok=True);ev=demand(10001,'constante',args.duration,args.scale,args.turns);route=train_dir/'demanda.rou.xml';write_routes(route,ev,args.emission_class)
    training=[]
    for g in ([39,43,45,49,53] if args.rapido else CANDIDATES):
        r=run(net,route,ev,args,train_dir/f'g{g}','B',g,10001,'constante');r['verde_testado']=g;training.append(r)
    write_csv(args.out/'treinamento.csv',training)
    feasible=[r for r in training if r['valido']]
    if not feasible:raise RuntimeError('Nenhum plano de treinamento concluiu todas as viagens sem falhas. Verifique os logs ou aumente --drain.')
    best=min(feasible,key=lambda r:r['atraso_medio_s']);fixed=best['verde_testado'];print(f'Plano B selecionado: {fixed}s / {AVAILABLE-fixed}s',flush=True)
    rows=[];series={}
    for profile in dict.fromkeys(args.perfis):
        for seed in seeds:
            d=args.out/profile/str(seed);d.mkdir(parents=True,exist_ok=True);ev=demand(seed,profile,args.duration,args.scale,args.turns);route=d/'demanda.rou.xml';write_routes(route,ev,args.emission_class)
            for s in SCENARIOS:
                r=run(net,route,ev,args,d/s,s,fixed if s in ('B','D') else HIST,seed,profile,trace=(seed==seeds[0]));rows.append(r);write_csv(args.out/'experimentos.csv',rows)
                if seed==seeds[0]:series[profile+'_'+s]=json.loads((d/s/'serie.json').read_text(encoding='utf-8'))
    agg=aggregate(rows);write_csv(args.out/'resumo.csv',agg)
    meta=dict(sumo=version,duracao_s=args.duration,drenagem_max_s=args.drain,seeds=seeds,escala_demanda=args.scale,conversoes_D_L=args.turns,classe_emissao=args.emission_class,verde_B=fixed,treinamento=[{'verde':r['verde_testado'],'atraso':r['atraso_medio_s'],'valido':r['valido']} for r in training],fluxos_vpa_h=FLOWS,perfis={k:PROFILES[k] for k in dict.fromkeys(args.perfis)},versao='3.0',geometria=validate_network(net),capacidade_referencia=capacity_reference(),pm10_fatores_g_km=PM10_FACTORS,massa_proxy_kg=MASS_KG)
    data=dict(meta=meta,resumo=agg,experimentos=rows,series=series)
    (args.out/'resultados.json').write_text(json.dumps(data,indent=2,ensure_ascii=False,allow_nan=False),encoding='utf-8')
    html=(ROOT/'painel_template.html').read_text(encoding='utf-8').replace('/*__DATA__*/null',json.dumps(data,ensure_ascii=False,allow_nan=False).replace('</','<\\/'))
    (args.out/'painel_atsc.html').write_text(html,encoding='utf-8')
    print('\nFinalizado. Abra:',args.out/'painel_atsc.html',flush=True)
    if not all(r['valido'] for r in rows): print('ATENÇÃO: há execuções inválidas/incompletas. Não use médias desses grupos.',flush=True)

if __name__=='__main__':
    try:main()
    except Exception as e:
        print('\nERRO:',e,file=sys.stderr);raise
