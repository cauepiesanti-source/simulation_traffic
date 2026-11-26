"""Ensaio de descarga saturada, independente da avaliação de controladores."""
from base_v3 import *
PRELOAD=100; BEGIN=110; END=140; STOP=145

def capacity_trial(net, folder, seed, phase, args):
    import traci
    folder.mkdir(parents=True,exist_ok=True)
    active=('D','E') if phase=='M' else ('L',)
    # Offer one vehicle/s/lane, greater than discharge rate; queues form on red.
    events=[]
    for t in range(STOP):
        for g in active:
            for lane in GROUPS[g]:events.append((float(t),f'{g}_{t:04d}_{lane}',g,lane,IN_EDGES[g]+' '+OUT_EDGES[g]))
    route=folder/'cap.rou.xml';write_routes(route,events,args.emission_class,args.tau,args.min_gap)
    add=ET.Element('additional')
    for g in active:
        for lane in GROUPS[g]:ET.SubElement(add,'inductionLoop',id=f'{g}_{lane}',lane=OUT_EDGES[g]+'_'+str(lane),pos='5',period='145',file=str(folder/'loops.xml'))
    af=folder/'loops.add.xml';write_xml(af,add)
    cmd=[binary('sumo'),'-n',str(net),'-r',str(route),'-a',str(af),'--seed',str(seed),'--step-length','1','--route-steps','0','--time-to-teleport','-1','--max-depart-delay','-1','--collision.action','warn','--no-step-log','true','--duration-log.disable','true','--log',str(folder/'sumo.log')]
    c=None;hits={g:set() for g in active};samples=Counter();counts=Counter();collisions=teleports=0;initial={}
    try:
        traci.start(cmd);c=traci.getConnection('default');groups=signal_groups(c.trafficlight.getControlledLinks('J'))
        c.trafficlight.setRedYellowGreenState('J','r'*len(groups))
        for t in range(STOP):
            if t==PRELOAD:
                initial={g:min(c.lane.getLastStepHaltingNumber(IN_EDGES[g]+'_'+str(i)) for i in GROUPS[g]) for g in active}
                c.trafficlight.setRedYellowGreenState('J',''.join('G' if x==phase else 'r' for x in groups))
            c.simulationStep();collisions+=c.simulation.getCollidingVehiclesNumber();teleports+=c.simulation.getStartingTeleportNumber()
            for g in active:
                for lane in GROUPS[g]:
                    for vid in c.inductionloop.getLastStepVehicleIDs(f'{g}_{lane}'):
                        if vid in hits[g]:continue
                        hits[g].add(vid)
                        if BEGIN<=t<END:counts[g]+=1;counts[g,lane]+=1
                if BEGIN<=t<END and all(c.lane.getLastStepHaltingNumber(IN_EDGES[g]+'_'+str(i))>0 for i in GROUPS[g]):samples[g]+=1
    finally:
        if c is not None:c.close()
    rows=[]
    for g in active:
        # Require queue on all three lanes at opening and >= 90% of measured steps.
        sat=counts[g]*3600/(END-BEGIN);coverage=samples[g]/(END-BEGIN)
        valid=initial[g]>=5 and coverage>=.9 and not collisions and not teleports and all(counts[g,i]>0 for i in GROUPS[g])
        rows.append(dict(grupo=g,semente=seed,verde_fase=phase,janela_s=END-BEGIN,veiculos=counts[g],faixa0=counts[g,0],faixa1=counts[g,1],faixa2=counts[g,2],fila_min_abertura=initial[g],fracao_fila_todas_faixas=coverage,saturacao_sim_vph=sat,saturacao_ref_vpa_h=SAT[g],desvio_pct=100*(sat/SAT[g]-1),tau=args.tau,min_gap=args.min_gap,colisoes=collisions,teleportes=teleports,valido=valid))
    return rows

def measure(net,folder,args):
    rows=[]
    for seed in ([20001] if args.rapido else [20001,20002,20003]):
        for phase in ('M','L'):
            rows.extend(capacity_trial(net,folder/f'{seed}_{phase}',seed,phase,args))
    write_csv(folder/'capacidade.csv',rows)
    summary=[]
    for g in FLOWS:
        subset=[r for r in rows if r['grupo']==g];valid=[r for r in subset if r['valido']];v=[r['saturacao_sim_vph'] for r in valid]
        mean=statistics.mean(v) if len(v)==len(subset) else None
        summary.append(dict(grupo=g,n=len(subset),n_validos=len(valid),saturacao_sim_vph=mean,saturacao_ref_vpa_h=SAT[g],desvio_pct=100*(mean/SAT[g]-1) if mean is not None else None,dp=statistics.stdev(v) if mean is not None and len(v)>1 else None,capacidade_nominal_sim_vph=mean*(32 if g=='L' else 45)/CYCLE if mean is not None else None))
    write_csv(folder/'capacidade_resumo.csv',summary)
    (folder/'capacidade.json').write_text(json.dumps(dict(janela_s=[BEGIN,END],pre_verde_s=PRELOAD,metodo='contagem de IDs únicos em laços a 5 m das saídas; fila continuamente disponível',tau=args.tau,min_gap=args.min_gap,resumo=summary,ensaios=rows),indent=2,ensure_ascii=False),encoding='utf-8')
    for r in summary:print('Capacidade:',r,flush=True)
    return summary
