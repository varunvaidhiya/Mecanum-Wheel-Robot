# Infrastructure & Tooling

This directory contains configuration files for development environments, containerization, and continuous integration.

## Directory Structure

*   **`docker/`**: Contains the `Dockerfile` used for both the development container and potential deployment images.
    *   `Dockerfile`: Multi-stage build (Base ROS 2 Jazzy image -> Dev image with tools).
*   **`devcontainers/`**: Contains the configuration for VS Code Dev Containers.
    *   `devcontainer.json`: Configures the VS Code environment, extensions, and mounts.
*   **`ci/`**: Scripts and tools for Continuous Integration.
    *   `test_all.sh`: A helper script to run `colcon build` and `colcon test` in the `robot_ws` directory.

## Getting Started (DevContainer)

1.  Ensure you have Docker and VS Code installed.
2.  Install the "Dev Containers" extension in VS Code.
3.  Open this project in VS Code.
4.  Run "Dev Containers: Reopen in Container" from the command palette.

## Running Tests Locally

You can use the helper script to run the full build and test suite:

```bash
bash infra/ci/test_all.sh
```
