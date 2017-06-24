Zubax document templates
========================

This repository contains templates for documents (LaTeX and such).
It is mostly inteneded for use as a submodule.

## Compiling LaTeX on Ubuntu

```bash
sudo apt-get install texlive-full lyx python-pygments
sudo apt-get install texmaker   # GUI IDE, optional
```

Then use the following helper script (which will probably require some modifications) to automate compilation:

```bash
#!/bin/bash

SRC=main

#rm -rf out &> /dev/null
mkdir out &> /dev/null
cp -fP *.bib out/ &> /dev/null

rm out/$SRC.pdf

pdflatex --halt-on-error --shell-escape -output-directory=out ../$SRC.tex
cd out
bibtex $SRC
#biber $SRC
cd ..
pdflatex --halt-on-error --shell-escape -output-directory=out ../$SRC.tex
pdflatex --halt-on-error --shell-escape -output-directory=out ../$SRC.tex

rm *.pdf*
```

Alternatively, use the Texmaker IDE.
In this case you will have to modify the compilation command so that `pdflatex` (or other LaTeX compiler)
is invoked with the option `-shell-escape`, otherwise the syntax coloring feature will not work.

## Editing notes

Avoid inclusion of complex graphics in EPS format, because it tends to be rendered poorly by `pdflatex`.
Prefer PDF instead.

### Wolfram Mathematica

When exporting graphics from Wolfram Mathematica, try PDF first, and if it doesn't work
(the PDF export feature often breaks on complex graphics), resort to EPS.
Overall the exporting forkflow should look like this:

1. Export the graphics from Mathematica to PDF (if it doesn't work, use EPS).
If annotations or other changes aren't necessary, the process ends here.
2. Open the exported file with Inkscape.
3. **Ungroup the graphics** (right click --> ungroup).
If the grraphics file is a plot that contains grid lines, move them all the way to the back
(select the grid lines and press "End" on the keyboard).
This is mandatory, otherwise the quality of the exported image may suffer.
3. Edit the graphics as necessary (e.g. add annotations), and
save it as Inkscape SVG with extension `.inkscape.svg`.
4. **Delete the temporary file obtained in the step 1**. There is no need to keep temporary files.
The Inkscape file, however, should be kept under version control in order to make editing easier later.
5. Export the file from Inkscape into PDF and include it into the LaTeX file.

It is best to avoid grid lines when exporting via EPS; or, if they are still required, try to avoid making them
dotted or dashed, because these features tend to look ugly when exported through EPS.
