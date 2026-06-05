EXAMPLES_DIR := $(patsubst %/,%,$(dir $(abspath $(lastword $(MAKEFILE_LIST)))))
SKELETON_DIR := $(EXAMPLES_DIR)/skeleton
GENERATED_DIR := $(EXAMPLES_DIR)/generated

EXAMPLE_PROJECTS := $(shell find $(EXAMPLES_DIR) -maxdepth 1 -mindepth 1 -type d \
	-not -name 'generated' -not -name 'skeleton' -not -name '*cpp*' \
	| xargs -I{} bash -c '[ -d {}/java ] && basename {}' | sort)

.PHONY: all examples clean $(EXAMPLE_PROJECTS)

all: examples

examples: clean $(EXAMPLE_PROJECTS)

clean:
	rm -rf $(GENERATED_DIR)/*

define make-example
.PHONY: $(1)
$(1):
	cp -r $(SKELETON_DIR) $(GENERATED_DIR)/$(1)
	mkdir -p $(GENERATED_DIR)/$(1)/src/main
	cp -r $(EXAMPLES_DIR)/$(1)/java $(GENERATED_DIR)/$(1)/src/main/java
	$(if $(wildcard $(EXAMPLES_DIR)/$(1)/deploy),cp -r $(EXAMPLES_DIR)/$(1)/deploy $(GENERATED_DIR)/$(1)/src/main/deploy)
endef

$(foreach proj,$(EXAMPLE_PROJECTS),$(eval $(call make-example,$(proj))))
