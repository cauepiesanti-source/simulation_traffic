#!/usr/bin/env python3
"""ATSC Curitiba v4: capacidade, controle atuado e sensibilidade à demanda."""
from base_v3 import *
from capacidade import measure
import re
DEFAULT_SCALES=[.8,1.,1.1,1.2,1.4]
TRAIN_SEEDS=[10001,10002]

def test():
    self_test()
    for pattern in ('empty','main','side','alternate','random'):
        a=Actuated(); rng=random.Random(123); states=[]
        for t in range(3600):
            pressure={'empty':{'M':0,'L':0},'main':{'M':30,'L':0},'side':{'M':0,'L':30},'alternate':{'M':20 if t%80<40 else 0,'L':0 if t%80<40 else 20},'random':{'M':rng.randrange(30),'L':rng.randrange(30)}}[pattern]
            states.append(a.step(t,pressure))
        runs=[]
        for i,x in enumerate(states):
            if i==0 or states[i-1]!=x:runs.append([i,1,x])
            else:runs[-1][1]+=1
        expected=[('M','G'),('M','y'),('-','r'),('L','G'),('L','y'),('-','r')]
        for i,(_,d,x) in enumerate(runs[:-1]):
            assert x==expected[i%6]
            assert (20<=d<=57 if x[1]=='G' else d==(4 if x[1]=='y' else 2))
        for group in ('M','L'):
            greens=[(t,d) for t,d,x in runs if x==(group,'G')]
            assert all(b-a-d<=69 for (a,d),(b,_) in zip(greens,greens[1:]))
    a=Actuated(); states=[a.step(t,{'M':0,'L':30}) for t in range(27)]
    assert states[19]==('M','G') and states[20]==('M','y') and states[24]==('-','r') and states[26]==('L','G')
    a=Actuated();assert a.step(0,{'M':30,'L':0})==('M','G')
    assert a.step(57,{'M':30,'L':0})==('M','y')
    print('OK: E — alternância, verde mínimo/máximo, amarelo, vermelho integral, limite sem verde e encerramento antecipado.')

def jsonwrite(path,data):
    tmp=path.with_suffix(path.suffix+'.tmp');tmp.write_text(json.dumps(data,ensure_ascii=False,allow_nan=False,indent=2),encoding='utf-8');tmp.replace(path)

def make_panel(path,data):
    template=(ROOT/'painel_template.html').read_text(encoding='utf-8')
    path.write_text(template.replace('/*__DATA__*/null',json.dumps(data,ensure_ascii=False,allow_nan=False).replace('</','<\\/')),encoding='utf-8')

def train(net,folder,args):
    rows=[]; seeds=TRAIN_SEEDS[:1] if args.rapido else TRAIN_SEEDS
    for seed in seeds:
        d=folder/str(seed);d.mkdir(parents=True,exist_ok=True);ev=demand(seed,'constante',args.duration,args.scale);route=d/'demanda.rou.xml';write_routes(route,ev,args.emission_class,args.tau,args.min_gap)
        for g in ([39,43,45,49,53] if args.rapido else CANDIDATES):
            r=run(net,route,ev,args,d/f'g{g}','B',g,seed,'constante');r['verde_testado']=g;rows.append(r)
    write_csv(folder/'treinamento.csv',rows)
    scores=[]
    for g in sorted({r['verde_testado'] for r in rows}):
        rs=[r for r in rows if r['verde_testado']==g];valid=all(r['valido'] for r in rs)
        scores.append(dict(verde=g,atraso=statistics.mean(r['atraso_medio_s'] for r in rs) if valid else None,valido=valid,n=len(rs)))
    feasible=[r for r in scores if r['valido']]
    if not feasible:raise RuntimeError('Nenhum candidato B válido; verificar logs e drenagem.')
    return min(feasible,key=lambda r:r['atraso'])['verde'],scores,rows

def verify_pairs(rows):
    for profile in set(r['perfil'] for r in rows):
        for seed in set(r['semente'] for r in rows):
            a=[r for r in rows if r['perfil']==profile and r['semente']==seed]
            if not a:continue
            if len({r['oferecidos'] for r in a})!=1:raise RuntimeError('Demanda diferente entre controles')
            if all(r['valido'] for r in a) and max(r['distancia_km'] for r in a)-min(r['distancia_km'] for r in a)>1e-5:raise RuntimeError('Distâncias divergentes')

def benchmark(net,folder,args,version,cap):
    folder.mkdir(parents=True,exist_ok=True);fixed,training,train_rows=train(net,folder/'treinamento',args);print(f'Plano B: {fixed}/{77-fixed} s',flush=True)
    seeds=args.seeds[:1] if args.rapido else args.seeds
    rows=[];series={}
    for profile in dict.fromkeys(args.perfis):
        for seed in seeds:
            d=folder/profile/str(seed);d.mkdir(parents=True,exist_ok=True);ev=demand(seed,profile,args.duration,args.scale);route=d/'demanda.rou.xml';write_routes(route,ev,args.emission_class,args.tau,args.min_gap)
            for s in SCENARIOS:
                r=run(net,route,ev,args,d/s,s,fixed if s in ('B','D') else HIST,seed,profile,trace=seed==seeds[0]);rows.append(r);write_csv(folder/'experimentos.csv',rows)
                if seed==seeds[0]:series[profile+'_'+s]=json.loads((d/s/'serie.json').read_text(encoding='utf-8'))
    verify_pairs(rows);agg=aggregate(rows);write_csv(folder/'resumo.csv',agg)
    meta=dict(versao='4.0',sumo=version,duracao_s=args.duration,drenagem_max_s=args.drain,seeds=seeds,escala_demanda=args.scale,conversoes_D_L=0,classe_emissao=args.emission_class,verde_B=fixed,treinamento=training,seeds_treino=TRAIN_SEEDS[:1] if args.rapido else TRAIN_SEEDS,fluxos_vpa_h=FLOWS,perfis={k:PROFILES[k] for k in dict.fromkeys(args.perfis)},geometria=validate_network(net),capacidade_referencia=capacity_reference(),capacidade_medida=cap,tau=args.tau,min_gap=args.min_gap,pm10_fatores_g_km=PM10_FACTORS,massa_proxy_kg=MASS_KG,rapido=args.rapido)
    data=dict(meta=meta,resumo=agg,experimentos=rows,series=series);jsonwrite(folder/'resultados.json',data)
    make_panel(folder/'painel_atsc.html',dict(capacidade=cap,escalas=[data],sensibilidade=sensitivity([data])))
    return data,train_rows

def sensitivity(datasets):
    keys=['atraso_medio_s','fila_media_veic','co2_kg','combustivel_kg','nox_g','pm_g','atraso_L_s','atraso_D_s','atraso_E_s','paradas_media','espera_entrada_s','pm10_nao_escape_g','energia_desaceleracao_MJ','desaceleracoes_fortes_s_veic','ciclo_medio_s']
    out=[]
    for data in datasets:
        m=data['meta'];rows=data['resumo'];expected=len(m['perfis']);baseline=[r for r in rows if r['cenario']=='A'];
        def means(rs,k):
            return statistics.mean(r[k] for r in rs) if len(rs)==expected and all(r['comparavel'] and r.get(k) is not None for r in rs) else None
        for s in SCENARIOS:
            rr=[r for r in rows if r['cenario']==s];row=dict(escala=m['escala_demanda'],demanda_vpa_h=sum(FLOWS.values())*m['escala_demanda'],cenario=s,n_perfis=expected,n_validos=sum(r['n_validos'] for r in rr),n=sum(r['n'] for r in rr))
            for k in keys:
                v=means(rr,k);b=means(baseline,k);row[k]=v;row[k+'_vs_A_pct']=100*(v/b-1) if v is not None and b is not None and b>0 else None
            out.append(row)
    return out

def paired(datasets):
    rows=[]
    for d in datasets:
        exp=d['experimentos'];lookup={(r['perfil'],r['semente'],r['cenario']):r for r in exp}
        for r in exp:
            if r['cenario']=='A':continue
            a=lookup.get((r['perfil'],r['semente'],'A'))
            if not a:continue
            valid=a['valido'] and r['valido'];row=dict(escala=d['meta']['escala_demanda'],perfil=r['perfil'],semente=r['semente'],cenario=r['cenario'],valido=valid)
            for k in ('atraso_medio_s','co2_kg','atraso_L_s','fila_media_veic'):
                row[k+'_delta']=r[k]-a[k] if valid else None
                row[k+'_pct']=100*(r[k]/a[k]-1) if valid and a[k]>0 else None
            rows.append(row)
    return rows

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--self-test',action='store_true');p.add_argument('--rapido',action='store_true',help='1 semente de avaliação/treino/capacidade; teste técnico, não conclusão');p.add_argument('--capacidade',action='store_true',help='somente ensaio de capacidade');p.add_argument('--sem-capacidade',action='store_true',help='omite ensaio no benchmark (já realizado separadamente)');p.add_argument('--gui',action='store_true');p.add_argument('--cenario',choices=SCENARIOS,default='E');p.add_argument('--perfil',choices=PROFILES,default='variavel');p.add_argument('--perfis',nargs='+',choices=PROFILES,default=list(PROFILES));p.add_argument('--scale',type=float);p.add_argument('--scales',nargs='+',type=float);p.add_argument('--seeds',nargs='+',type=int,default=[45,46,47,48,49]);p.add_argument('--duration',type=int,default=3600);p.add_argument('--drain',type=int,default=3600);p.add_argument('--tau',type=float,default=1.0);p.add_argument('--min-gap',type=float,default=2.5);p.add_argument('--emission-class',default='HBEFA3/PC_G_EU4');p.add_argument('--verde-fixo',type=int,default=45);p.add_argument('--out',type=Path,default=ROOT/'resultados_v4');args=p.parse_args()
    test()
    if args.self_test:return
    if args.scale is not None and args.scales:p.error('Escolha --scale ou --scales')
    scales=args.scales or ([args.scale] if args.scale is not None else ([1.] if args.rapido or args.gui else DEFAULT_SCALES));args.scale=scales[0]
    if len(set(scales))!=len(scales) or any(not .1<=x<=2 for x in scales) or args.duration<360 or args.drain<60 or not 1<=args.tau<=3 or not 0<=args.min_gap<=10 or not 20<=args.verde_fixo<=57:p.error('Parâmetros inválidos (tau >= passo de 1 s)')
    if len(set(args.seeds))!=len(args.seeds) or any(s<0 or s>2147483647 or s in TRAIN_SEEDS+[20001,20002,20003] for s in args.seeds):p.error('Sementes inválidas ou reservadas a treino/capacidade')
    if args.gui and len(scales)!=1:p.error('GUI aceita uma escala')
    if args.capacidade and (args.gui or args.sem_capacidade):p.error('Modos incompatíveis')
    args.out=args.out.resolve()
    if args.out.exists() and any(args.out.iterdir()):raise RuntimeError('Pasta não vazia. Use outro --out para preservar resultados.')
    args.out.mkdir(parents=True,exist_ok=True)
    import traci
    version=subprocess.check_output([binary('sumo'),'--version'],text=True,encoding='utf-8',errors='replace').splitlines()[0];m=re.search(r'(\d+)\.(\d+)',version)
    if not m or tuple(map(int,m.groups()))<(1,14):raise RuntimeError('Requer SUMO >=1.14: '+version)
    net=build_network(args.out/'rede');jsonwrite(args.out/'capacidade_referencia.json',capacity_reference())
    if args.gui:
        seed=args.seeds[0];ev=demand(seed,args.perfil,args.duration,args.scale);d=args.out/'gui';d.mkdir();route=d/'demanda.rou.xml';write_routes(route,ev,args.emission_class,args.tau,args.min_gap);run(net,route,ev,args,d,args.cenario,args.verde_fixo if args.cenario in ('B','D') else HIST,seed,args.perfil,True,True);return
    cap=[] if args.sem_capacidade else measure(net,args.out/'capacidade',args)
    if args.capacidade:return
    datasets=[];all_training=[]
    for scale in scales:
        args.scale=scale;print(f'\nESCALA {scale:g}',flush=True)
        folder=args.out/f'escala_{scale:g}';data,tr=benchmark(net,folder,args,version,cap);datasets.append(data);all_training.extend(dict(escala=scale,**r) for r in tr)
        sens=sensitivity(datasets);write_csv(args.out/'sensibilidade.csv',sens);write_csv(args.out/'comparacoes_pareadas.csv',paired(datasets));write_csv(args.out/'treinamento.csv',all_training);write_csv(args.out/'resumo.csv',[dict(escala=d['meta']['escala_demanda'],**r) for d in datasets for r in d['resumo']]);write_csv(args.out/'experimentos.csv',[dict(escala=d['meta']['escala_demanda'],**r) for d in datasets for r in d['experimentos']]);combined=dict(versao='4.0',capacidade=cap,escalas=datasets,sensibilidade=sens);jsonwrite(args.out/'resultados.json',combined);make_panel(args.out/'painel_atsc.html',combined)
    print('Finalizado:',args.out/'painel_atsc.html',flush=True)
    if any(not r['valido'] for d in datasets for r in d['experimentos']):print('ATENÇÃO: grupos incompletos sem médias. Veja experimentos.csv.')

if __name__=='__main__':
    try:main()
    except Exception as exc:print('ERRO:',exc,file=sys.stderr);raise
