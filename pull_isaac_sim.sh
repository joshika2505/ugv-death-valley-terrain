#!/usr/bin/env bash
IMAGE="nvcr.io/nvidia/isaac-sim:4.2.0"
echo "Starting resilient download for $IMAGE..."

count=1
until docker pull "$IMAGE"; do
    echo "[$count] Connection interrupted or timed out. Resuming in 2 seconds..."
    ((count++))
    sleep 2
done

echo "🎉 Isaac Sim Docker image downloaded successfully!"
