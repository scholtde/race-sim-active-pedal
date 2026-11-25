import math
CPM=2048/3
def curve(F, bite=True, p=20, db=1.0, bk=2.5, bm=10, top=44, park=2, gamma=0.6, a=0.5):
    if F<=db: return park
    if bite:
        if F<bk: return park+(F-db)/(bk-db)*(bm-park)
        u=min(1,(F-bk)/(p-bk)); t=(-a+math.sqrt(a*a+4*(1-a)*u))/(2*(1-a)); return bm+t*(top-bm)
    u=min(1,(F-db)/(p-db)); return park+u**gamma*(top-park)
def sim(Feq=8, kh=1.0, tau=0, comp=0.8, v0=3, drag=4.4, vmax=400, hyst=0.25, bite=True, T=3000, damp=0):
    # find hand preload A so that equilibrium (no motion) gives Feq
    xeq=curve(Feq,bite); A=Feq+kh*xeq
    x=2.0; v=0.0  # mm, mm/ms
    filt=0; fb=0; xc=x; hold=False; hist=[]
    for t in range(T):
        Fh=max(0,A-kh*x)
        vc=v*CPM  # counts/ms
        cell=Fh - (drag if vc>0.5 else 0)*min(1,abs(vc)/1.0)   # arm friction shows when moving fwd
        if t%12==0: filt+=0.2*(cell-filt)
        sv=max(-1,min(1,vc/v0)); est=filt-comp*(-drag*sv if sv>0 else 0) - damp*vc
        if est<1: fb=est
        elif est>fb+hyst: fb=est-hyst
        elif est<fb-hyst: fb=est+hyst
        tgt=curve(fb,bite)
        if tau: xc+= (tgt-xc)*min(1,1/tau)
        else: xc=tgt
        err=(xc-x)*CPM; ae=abs(err)
        if ae <= (205 if hold else 68): hold=True; cmd=0
        else:
            hold=False; cmd=min(vmax,max(70,ae/9))*(1 if err>0 else -1)
        if cmd==0: vt=0; tm=8
        else: vt=(0.233*abs(cmd)-4.7)*(1 if cmd>0 else -1)/CPM; tm=20
        v+=(vt-v)/tm
        x+=v
        hist.append((x,Fh))
    xs=[h[0] for h in hist[T//2:]]; fs=[h[1] for h in hist[T//2:]]
    return max(xs)-min(xs), max(fs)-min(fs), sum(xs)/len(xs)
print("Feq kh  | old(V200,nobite,nocomp) | new as run | +tau60 comp.5 v0=10 | tau100 | tau60 nocomp")
for Feq in (2,5,8,14):
  for kh in (0.5,1,2):
    r=[sim(Feq,kh,tau=0,comp=0,vmax=200,bite=False,hyst=0),
       sim(Feq,kh),
       sim(Feq,kh,tau=60,comp=0.5,v0=10),
       sim(Feq,kh,tau=100,comp=0.5,v0=10),
       sim(Feq,kh,tau=60,comp=0,v0=10)]
    print(Feq,kh,' | '.join("%5.2fmm %4.1fkg"%(a,b) for a,b,c in r))
print()
cfgs={"nocomp tau0":dict(tau=0,comp=0),"nocomp tau40":dict(tau=40,comp=0),"nocomp tau60":dict(tau=60,comp=0),
      "nocomp tau100":dict(tau=100,comp=0),"nocomp tau60 hyst.5":dict(tau=60,comp=0,hyst=0.5),
      "nocomp tau60 damp.02":dict(tau=60,comp=0,damp=0.02),"nocomp tau100 damp.02":dict(tau=100,comp=0,damp=0.02),
      "nocomp tau150":dict(tau=150,comp=0)}
for name,c in cfgs.items():
    worst=0; tot=0
    for Feq in (2,3.5,5,8,11,14,18):
        for kh in (0.5,1,1.5,2):
            a,b,m=sim(Feq,kh,**c); worst=max(worst,a); tot+=a
    print("%-24s worst %5.2f mm  mean %5.2f mm"%(name,worst,tot/28))
