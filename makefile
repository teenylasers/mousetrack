default:
-include /Users/jq/eggshell/common.mk

DIRS = build

MAKEFLAGS += -R
MY_MAKEFLAGS = -C build -f ../makefile

.PHONY: dirs docs clean

dirs:
	mkdir -p $(DIRS)

docs: dirs
	$(MAKE) $(MY_MAKEFLAGS) docs2

docs2:
	$(DOCCER) ../docs/mousetrack.doc | sed 's@__MATHJAX__@$(MATHJAX_PUBLIC)@' | sed 's@__TITLE__@mousetrack@' > mousetrack.html

clean:
	rm -rf build
