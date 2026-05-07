.PHONY: build_pc run_pc build_jetson run_jetson

build_pc:
	@echo "Building PC Docker environment..."
	docker compose build fsd-dev

run_pc:
	@echo "Running PC Docker environment..."
	docker compose up -d fsd-dev
	docker exec -it fsd_dev bash

build_jetson:
	@echo "Building Jetson Docker environment..."
	docker compose -f docker-compose.jetson.yml build fsd-jetson

run_jetson:
	@echo "Running Jetson Docker environment..."
	docker compose -f docker-compose.jetson.yml up -d fsd-jetson
	docker exec -it fsd_dev_jetson bash
