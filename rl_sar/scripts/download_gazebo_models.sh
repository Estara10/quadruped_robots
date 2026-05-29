#!/bin/bash

# Automatic download script for Gazebo models
# Downloads only required models (sun, ground_plane) using sparse checkout
# Usage: ./download_gazebo_models.sh

set -e

# Get script directory
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

# Load common utilities
source "${SCRIPT_DIR}/common.sh"

# Configuration
GAZEBO_MODEL_DIR="$HOME/.gazebo/models"
REQUIRED_MODELS=("sun" "ground_plane")
GAZEBO_MODELS_REPO="https://github.com/osrf/gazebo_models.git"

# Check if all required models exist
are_models_valid() {
    for model in "${REQUIRED_MODELS[@]}"; do
        if [ ! -d "$GAZEBO_MODEL_DIR/$model" ]; then
            return 1
        fi
    done
    return 0
}

# Download models using sparse checkout
download_models() {
    print_info "Downloading Gazebo models (sun, ground_plane)..."
    mkdir -p "$GAZEBO_MODEL_DIR"
    cd "$GAZEBO_MODEL_DIR"

    if [ -d ".git" ]; then
        # Already a git repo, update sparse checkout
        print_info "Updating existing Gazebo models..."
        git sparse-checkout set "${REQUIRED_MODELS[@]}"
        git pull --depth 1 origin master || true
    else
        # Fresh clone with sparse checkout
        print_info "Cloning Gazebo models repository..."
        git clone --depth 1 --filter=blob:none --sparse \
            "$GAZEBO_MODELS_REPO" .
        git sparse-checkout set "${REQUIRED_MODELS[@]}"
    fi
}

# Main
print_header "[Gazebo Models Setup]"

if are_models_valid; then
    print_success "Gazebo models already exist at $GAZEBO_MODEL_DIR"
    exit 0
fi

if ! is_online; then
    print_warning "Network unavailable, skipping Gazebo models download"
    print_info "You can manually download models later:"
    print_info "  git clone https://github.com/osrf/gazebo_models.git ~/.gazebo/models"
    exit 0
fi

download_models

if are_models_valid; then
    print_success "Gazebo models setup completed!"
else
    print_error "Failed to download Gazebo models"
    exit 1
fi
