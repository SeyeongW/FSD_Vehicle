#!/bin/bash
echo "Running PC Docker environment..."
docker compose up -d fsd-dev
echo "Attaching to container..."
docker exec -it fsd_dev bash
