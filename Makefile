.PHONY: build_pc run_pc

build_pc:
	@echo "Building PC Docker environment..."
	docker-compose build ugv-dev

run_pc:
	@echo "Running PC Docker environment..."
	docker-compose up -d ugv-dev
	docker exec -it ugv_dev bash
