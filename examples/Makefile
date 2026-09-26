EXAMPLES_DIR := $(patsubst %/,%,$(dir $(abspath $(lastword $(MAKEFILE_LIST)))))
# Commands v2 and v3 examples, each with the skeleton they are generated from.
COMMANDS2_DIR := $(EXAMPLES_DIR)/commands2
COMMANDS3_DIR := $(EXAMPLES_DIR)/commands3
SKELETON2_DIR := $(COMMANDS2_DIR)/simple_robot-2027
SKELETON3_DIR := $(COMMANDS3_DIR)/simple_robot-2027
# Generated projects must stay two levels below the repository root so the skeleton's YAMS_TESTING
# source paths (../../../yams/java) resolve.
GENERATED_DIR := $(EXAMPLES_DIR)/generated

# Every folder with a java/ subfolder is an example. Commands v3 targets are prefixed with
# "commands3-" so they do not clash with the commands v2 example of the same name.
find-examples = $(shell find $(1) -maxdepth 1 -mindepth 1 -type d \
	-not -name 'generated' -not -name '*cpp*' \
	| xargs -I{} bash -c '[ -d {}/java ] && basename {}' | sort)
EXAMPLE_PROJECTS := $(call find-examples,$(COMMANDS2_DIR))
EXAMPLE3_PROJECTS := $(addprefix commands3-,$(call find-examples,$(COMMANDS3_DIR)))

.PHONY: all examples build clean simulate $(EXAMPLE_PROJECTS) $(EXAMPLE3_PROJECTS)

all: examples

examples: clean $(EXAMPLE_PROJECTS) $(EXAMPLE3_PROJECTS)

build: examples
	@for dir in $(GENERATED_DIR)/*/; do \
		echo "Building $$dir..."; \
		chmod +x "$$dir/gradlew"; \
		(cd "$$dir" && ./gradlew build) || exit 1; \
	done

simulate: examples
	@for dir in $(GENERATED_DIR)/*/; do \
		echo "Simulating $$dir..."; \
		chmod +x "$$dir/gradlew"; \
		(cd "$$dir" && ./gradlew simulateJava) || exit 1; \
	done

clean:
	rm -rf $(GENERATED_DIR)/*

# $(1): target and generated folder name, $(2): example source folder, $(3): skeleton folder.
define make-example
.PHONY: $(1)
$(1):
	@echo "Generating '$(1)' example project"
	@mkdir -p $(GENERATED_DIR)
	@cp -r $(3) $(GENERATED_DIR)/$(1)
	@rm -rf $(GENERATED_DIR)/$(1)/build $(GENERATED_DIR)/$(1)/bin $(GENERATED_DIR)/$(1)/.gradle
	@rm -rf $(GENERATED_DIR)/$(1)/src/main/java
	@mkdir -p $(GENERATED_DIR)/$(1)/src/main
	@cp -r $(2)/java $(GENERATED_DIR)/$(1)/src/main/java
	@$(if $(wildcard $(2)/deploy),rm -rf $(GENERATED_DIR)/$(1)/src/main/deploy && cp -r $(2)/deploy $(GENERATED_DIR)/$(1)/src/main/deploy)
	@$(if $(wildcard $(2)/build.gradle),cp $(2)/build.gradle $(GENERATED_DIR)/$(1)/build.gradle)
	@$(if $(wildcard $(2)/vendordeps),cp -r $(2)/vendordeps/. $(GENERATED_DIR)/$(1)/vendordeps/)
endef

$(foreach proj,$(EXAMPLE_PROJECTS),$(eval $(call make-example,$(proj),$(COMMANDS2_DIR)/$(proj),$(SKELETON2_DIR))))
$(foreach proj,$(EXAMPLE3_PROJECTS),$(eval $(call make-example,$(proj),$(COMMANDS3_DIR)/$(patsubst commands3-%,%,$(proj)),$(SKELETON3_DIR))))
