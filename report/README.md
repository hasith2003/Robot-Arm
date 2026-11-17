# EN3563 report — Overleaf / LaTeX

The revised report includes explicit link/frame numbering, a dimensioned
Figure 1 rendered from the actual GP7 URDF visual meshes with joint-axis
labels in zero and recorded bent configurations, the flange/TCP distinction, the signed
Z-Y-Z wrist rotation, the wrist-centre equation, and the distinction between
quintic Cartesian timing and cubic joint interpolation. The D-H table and
frame locations are unchanged. The compiled report is 8 pages.

1. Replace the group index-number placeholders near the top of `main.tex`.
   Remove unused member entries. Add your repository URL, or remove its
   optional placeholder near the end.
2. In Overleaf, choose **New Project → Upload Project** and upload
   `gp7_report_overleaf.zip` from the workspace root.
3. Select `main.tex` as the main document and pdfLaTeX as the compiler.
4. Recompile and download the PDF. Submit the simulation video separately.

For local compilation, install a TeX distribution (TeX Live or MiKTeX) and run
`pdflatex -interaction=nonstopmode -halt-on-error main.tex` twice in this folder.
Alternatively, a standalone Tectonic compiler can run `tectonic main.tex`.

The VS Code LaTeX Workshop extension can automate compilation once the TeX
distribution is installed; the extension itself is not a compiler.

The figure files are bundled. `results.json` contains the measured values used
in the report. The LaTeX uses standard packages and does not require shell
escape or minted. The source intentionally documents the selected reachable
destination, fixed-joint grasp abstraction and actual control architecture.
