"""Erzeugt fig-follower-zustandsdiagramm.png/.svg. Aufruf: python erzeuge_fig-follower-zustandsdiagramm.py fig-follower-zustandsdiagramm"""
import sys
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch

INK="#17212B"; MUTED="#4B5563"; BLUE="#305B78"; LBLUE="#E7EEF4"; ORANGE="#E87520"; LOR="#FBE3D1"
plt.rcParams["font.family"]="Helvetica"
fig, ax = plt.subplots(figsize=(12, 6.2))
ax.set_xlim(-0.2, 11.8); ax.set_ylim(0.2, 6.4); ax.axis("off")
W, H = 1.5, 0.6
pos = {"WARTEN":(0.8,3.2), "ANFAHREN":(3.3,3.2), "FOLGEN":(5.8,3.2), "ABSENKEN":(8.3,3.2), "GREIFEN":(10.8,3.2),
       "HEBEN":(10.8,1.0), "ABLEGEN":(8.3,1.0), "LOESEN":(5.8,1.0), "ABBRUCH":(5.8,5.3)}
for n,(x,y) in pos.items():
    ab = n=="ABBRUCH"
    ax.add_patch(FancyBboxPatch((x-W/2,y-H/2), W, H, boxstyle="round,pad=0.02,rounding_size=0.12",
                 fc=LOR if ab else LBLUE, ec=ORANGE if ab else BLUE, lw=1.4, zorder=3))
    ax.text(x, y, n, ha="center", va="center", fontsize=11, color=INK, weight="bold", zorder=4)

def arrow(a, b, label="", lp=None, rad=0.0, color=MUTED, ls="-", ha="center", tc=INK):
    ax.add_patch(FancyArrowPatch(a, b, arrowstyle="-|>", mutation_scale=12, color=color, lw=1.2,
                 linestyle=ls, connectionstyle=f"arc3,rad={rad}", zorder=2, shrinkA=0, shrinkB=0))
    if label:
        ax.text(lp[0], lp[1], label, ha=ha, va="center", fontsize=9.5, color=tc, zorder=5)

def R(n): x,y=pos[n]; return (x+W/2, y)
def L(n): x,y=pos[n]; return (x-W/2, y)
def T(n): x,y=pos[n]; return (x, y+H/2)
def B(n): x,y=pos[n]; return (x, y-H/2)
def mid(a,b,dy): return ((a[0]+b[0])/2, a[1]+dy)

for a,b,lab in [("WARTEN","ANFAHREN","Ziel\ngewählt"),("ANFAHREN","FOLGEN","Klotz in\nGreifzone"),
                ("FOLGEN","ABSENKEN","Abweichung\nstabil klein"),("ABSENKEN","GREIFEN","Greifhöhe\nerreicht")]:
    arrow(R(a), L(b), lab, (mid(R(a),L(b),0.5)[0]-(0.12 if a=="WARTEN" else 0), R(a)[1]+0.5))
arrow(B("GREIFEN"), T("HEBEN"), "Klotz\ngehalten", (10.95, 2.1), ha="left")
for a,b,lab in [("HEBEN","ABLEGEN","Freihöhe\nerreicht"),("ABLEGEN","LOESEN","über der\nKiste")]:
    arrow(L(a), R(b), lab, mid(L(a),R(b),-0.5))
arrow(L("LOESEN"), B("WARTEN"), "Greifer offen", (2.2, 1.35), rad=-0.2)

# Abbruch
gx0, gx1 = pos["ANFAHREN"][0]-W/2-0.1, pos["GREIFEN"][0]+W/2+0.2
ax.add_patch(FancyBboxPatch((gx0, 2.7), gx1-gx0, 1.4, boxstyle="round,pad=0.02,rounding_size=0.15",
             fc="none", ec=ORANGE, lw=1.1, ls="--", zorder=1))
arrow((5.8, 4.12), B("ABBRUCH"), color=ORANGE, ls="--")
ax.text(6.8, 5.3, "Greifebene überschritten, Ziel entzogen,\nZeitüberschreitung oder Fehlgriff",
        ha="left", va="center", fontsize=9.5, color=ORANGE)
arrow((5.8, 6.2), T("ABBRUCH"), "Start", (5.9, 6.05), ha="left")
arrow(L("ABBRUCH"), T("WARTEN"), "senkrecht auf Freihöhe", (2.0, 5.25), rad=0.25)
fig.savefig(f"{sys.argv[1]}.png", dpi=200, bbox_inches="tight", facecolor="white")
fig.savefig(f"{sys.argv[1]}.svg", bbox_inches="tight", facecolor="white")
