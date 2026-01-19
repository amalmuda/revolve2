import sqlite3, glob, os
def mean(lst): return sum(lst)/len(lst) if lst else 0

for robot, rdir in [("spider","spider_v2"),("gecko","gecko_v2"),("arachnid","arachnid_v2"),("tripod","tripod_v1")]:
    data = []
    for db in glob.glob(f"results/{rdir}_bounds1_pop25_25seeds/{robot}_m1_power_bounds1_lambda*/run_*.sqlite"):
        try:
            c = sqlite3.connect(db).cursor()
            c.execute("SELECT contact_m1, stability FROM individual i JOIN population p ON i.population_id=p.id JOIN generation g ON g.population_id=p.id WHERE g.generation_index=(SELECT MAX(generation_index) FROM generation) ORDER BY i.fitness DESC LIMIT 1")
            r = c.fetchone()
            if r: data.append((r[0]*100 if r[0] else 0, r[1] if r[1] else 0))
        except: pass
    h = [d[1] for d in data if d[0]>=40]
    m = [d[1] for d in data if 20<=d[0]<40]
    l = [d[1] for d in data if d[0]<20]
    print(f"{robot}: low={mean(l):.3f} med={mean(m):.3f} high={mean(h):.3f}")
