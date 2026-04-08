#!/bin/bash
echo "Running PC Docker environment..."
docker-compose up -d ugv-dev
echo "Attaching to container..."
docker exec -it ugv_dev bash
