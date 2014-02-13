#  Создание файла для печати
name=timer
suffix=B_cu
psfile=$(name)-$(suffix).ps 
epsfile=$(name)-$(suffix).eps 
pdffile=$(name)-$(suffix).pdf
printfile=print.pdf
 
# Правило по умолчанию - генерация финального документа
all: $(printfile) 

$(printfile):  $(pdffile)
	@echo Make print version
	pdflatex print.tex
 
$(pdffile): $(epsfile) 
	eps2pdf /f=$(epsfile)
	
$(epsfile):
	ps2epsi.exe $(psfile) $(epsfile) 
 
