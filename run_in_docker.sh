#!/bin/bash
set -e

# Разрешаем локальным контейнерам подключаться к X11-серверу для GUI
xhost +local:root >/dev/null 2>&1 || true
xhost +local:docker >/dev/null 2>&1 || true

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
IMAGE_NAME="ros2_seminars_env:humble"

# Если локальный образ с зависимостями не собран, собираем его из .devcontainer
if ! docker image inspect "$IMAGE_NAME" >/dev/null 2>&1; then
    echo "Локальный образ $IMAGE_NAME не найден. Выполняется сборка из .devcontainer/Dockerfile..."
    docker build -t "$IMAGE_NAME" -f "$WORKSPACE_DIR/.devcontainer/Dockerfile" "$WORKSPACE_DIR"
fi

echo "Запуск ROS 2 Humble окружения в Docker..."
docker run -it --rm \
    --net=host \
    --ipc=host \
    --privileged \
    -e DISPLAY="$DISPLAY" \
    -e QT_X11_NO_MITSHM=1 \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v /dev:/dev \
    -v "$WORKSPACE_DIR:/workspaces/ros2_seminars_ws" \
    -w /workspaces/ros2_seminars_ws \
    "$IMAGE_NAME" \
    /bin/bash