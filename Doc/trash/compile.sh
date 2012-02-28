#! /bin/bash

## First compile title page
#cd 00_Prae/Title/
#latex title.tex
#dvips -o title-pics.ps title.dvi
#ps2pdf title-pics.ps
#pdflatex title.tex
#cp title.pdf ../
#cd ../..

## Now compile document a total of 4 times
#latex master.tex
#dvips -o master-pics.ps master.dvi
#ps2pdf master-pics.ps
#pdflatex -draftmode master.tex
#bibtex master
#pdflatex -draftmode master.tex
#pdflatex master.tex

## Copy log file to directory above, if required [provide any commandline argument to activate]
#if [ 0 -ne $# ]
#then
	#cp 00_Prae/Title/title.log ../
	#cp master.log ../
#fi

## Delete unnecessary files
#rm 00_Prae/title.pdf 00_Prae/Title/title.[abdlopt][^e][a-z] 00_Prae/Title/*-pics.* master.[abdlot][^e][a-z] *-pics.*
#find -name '*.aux' | xargs rm

latex spec.tex
dvips -o spec-pics.ps spec.dvi
ps2pdf spec-pics.ps
pdflatex -draftmode spec.tex
pdflatex spec.tex
rm -f spec.[abdlot][^e]? spec.ps *-pics.*
