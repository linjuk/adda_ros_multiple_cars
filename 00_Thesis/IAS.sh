#!/bin/bash
# CLEAN UP
shopt -s extglob nocaseglob
for file in !(@(*.sty|*.pdf|*.tex|*.sh)); do
    [[ -f "${file}" ]] && files+=( "${file}" )
done
(( ${#files[@]} )) && rm "${files[@]}"

# COMPILE
pdflatex thesis.tex
pdflatex thesis.tex
makeglossaries thesis
makeglossaries thesis
bibtex thesis
pdflatex thesis.tex
pdflatex thesis.tex

# CLEAN UP
shopt -s extglob nocaseglob
for file in !(@(*.sty|*.pdf|*.tex|*.sh)); do
    [[ -f "${file}" ]] && files+=( "${file}" )
done
(( ${#files[@]} )) && rm "${files[@]}"
