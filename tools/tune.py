exec(open("loop.py").read().split("print(\"Feq kh")[0])
def score(**c):
    worst=0; tot=0; n=0
    for Feq in (1.8,2.3,3.5,5,8,11,14,18):
        for kh in (0.5,1,1.5,2,3):
            a,b,m=sim(Feq,kh,**c); worst=max(worst,a); tot+=a; n+=1
    return worst, tot/n
for tau in (0,30,60,100):
    for damp in (0.01,0.02,0.03,0.05):
        for comp in (0,0.5):
            w,m=score(tau=tau,comp=comp,v0=10,damp=damp)
            print("tau %3d damp %.2f comp %.1f  worst %5.2f mean %5.2f"%(tau,damp,comp,w,m))
