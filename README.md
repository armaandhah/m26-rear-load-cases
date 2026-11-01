# M26 Rear Load Cases

Interactive, step-by-step calculator for the UCLA Bruin Baja M26 rear suspension (trailing arm + 2 camber links + shock).

**Live site:** https://armaandhah.github.io/m26-rear-load-cases/

What it does:
1. Static corner load and weight transfer per g
2. Load cases → 3D tire contact-patch force
3. CAD hardpoints → member unit vectors and lever arms
4. 6×6 equilibrium solve (ΣF = 0, ΣM about TA pivot = 0)
5. Upright → trailing arm joint force and moment
6. Camber link yield and Johnson/Euler buckling checks
7. Peak design loads at every joint (for lug/bolt sizing)

Everything is in `index.html` (single file, no build step). `engine.js` is the same calc engine as a standalone module for testing with Node.
Equations render with MathJax from jsDelivr.

Replaces `Rear_Load_Cases_4.xlsx`. See step 08 on the site for the bugs fixed from the old sheet.
