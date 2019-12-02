# Makefile for slides from Markdown using pandoc
#
PANDOC := pandoc
PDFLATEX := pdflatex
BUILD_DIR := build
BEAMER_TEMPLATE = UC_Davis_template.tex
SRC := model-based_control.md
OUTPUT := model-based_control.pdf

$(OUTPUT) : $(SRC)
	@echo "Building $@ Beamer presentation with pandoc."
	@mkdir -p $(BUILD_DIR)
	@sed '/<!--/,/-->/d' $< | \
		$(PANDOC) --from markdown+latex_macros+link_attributes --to beamer \
			--highlight-style tango \
			--template=$(BEAMER_TEMPLATE) --pdf-engine=$(PDFLATEX) --output $@
