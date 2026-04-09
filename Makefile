.PHONY: build_pc run_pc

build_pc:
	@echo "Building PC Docker environment..."
	docker compose build fsd-dev

run_pc:
	@echo "Running PC Docker environment..."
	docker compose up -d fsd-dev
	docker exec -it fsd_dev bash
