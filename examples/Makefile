EXAMPLES_DIR := $(patsubst %/,%,$(dir $(abspath $(lastword $(MAKEFILE_LIST)))))
# Commands v2 examples and the skeleton they are generated from.
COMMANDS2_DIR := $(EXAMPLES_DIR)/commands2
SKELETON_DIR := $(COMMANDS2_DIR)/simple_robot-2027
# Generated projects must stay two levels below the repository root so the skeleton's YAMS_TESTING
# source paths (../../../yams/java) resolve.
GENERATED_DIR := $(EXAMPLES_DIR)/generated

EXAMPLE_PROJECTS := $(shell find $(COMMANDS2_DIR) -maxdepth 1 -mindepth 1 -type d \
	-not -name 'generated' -not -name '*cpp*' \
	| xargs -I{} bash -c '[ -d {}/java ] && basename {}' | sort)

.PHONY: all examples build clean simulate $(EXAMPLE_PROJECTS)

all: examples

examples: clean $(EXAMPLE_PROJECTS)

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

define make-example
.PHONY: $(1)
$(1):
	@echo "Generating '$(1)' example project"
	@mkdir -p $(GENERATED_DIR)
	@cp -r $(SKELETON_DIR) $(GENERATED_DIR)/$(1)
	@rm -rf $(GENERATED_DIR)/$(1)/build $(GENERATED_DIR)/$(1)/bin $(GENERATED_DIR)/$(1)/.gradle
	@rm -rf $(GENERATED_DIR)/$(1)/src/main/java
	@mkdir -p $(GENERATED_DIR)/$(1)/src/main
	@cp -r $(COMMANDS2_DIR)/$(1)/java $(GENERATED_DIR)/$(1)/src/main/java
	@$(if $(wildcard $(COMMANDS2_DIR)/$(1)/deploy),rm -rf $(GENERATED_DIR)/$(1)/src/main/deploy && cp -r $(COMMANDS2_DIR)/$(1)/deploy $(GENERATED_DIR)/$(1)/src/main/deploy)
	@$(if $(wildcard $(COMMANDS2_DIR)/$(1)/build.gradle),cp $(COMMANDS2_DIR)/$(1)/build.gradle $(GENERATED_DIR)/$(1)/build.gradle)
	@$(if $(wildcard $(COMMANDS2_DIR)/$(1)/vendordeps),cp -r $(COMMANDS2_DIR)/$(1)/vendordeps/. $(GENERATED_DIR)/$(1)/vendordeps/)
endef

$(foreach proj,$(EXAMPLE_PROJECTS),$(eval $(call make-example,$(proj))))
