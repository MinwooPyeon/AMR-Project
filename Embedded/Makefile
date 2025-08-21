.PHONY: help install test clean build docker-build docker-run docker-stop lint format

# Default target
help:
	@echo "AMR System - Available commands:"
	@echo "  install      - Install dependencies"
	@echo "  test         - Run all tests"
	@echo "  test-unit    - Run unit tests only"
	@echo "  test-integration - Run integration tests only"
	@echo "  test-e2e     - Run E2E tests only"
	@echo "  lint         - Run linting"
	@echo "  format       - Format code"
	@echo "  clean        - Clean build artifacts"
	@echo "  build        - Build the project"
	@echo "  docker-build - Build Docker image"
	@echo "  docker-run   - Run with Docker Compose"
	@echo "  docker-stop  - Stop Docker containers"

# Install dependencies
install:
	pip install -r requirements.txt

# Run all tests
test:
	pytest tests/ -v --cov=. --cov-report=html

# Run unit tests only
test-unit:
	pytest tests/unit/ -v

# Run integration tests only
test-integration:
	pytest tests/integration/ -v

# Run E2E tests only
test-e2e:
	pytest tests/e2e/ -v

# Run linting
lint:
	flake8 . --max-line-length=100 --exclude=venv,__pycache__,*.pyc
	mypy . --ignore-missing-imports

# Format code
format:
	black . --line-length=100
	isort .

# Clean build artifacts
clean:
	find . -type f -name "*.pyc" -delete
	find . -type d -name "__pycache__" -delete
	find . -type d -name "*.egg-info" -exec rm -rf {} +
	rm -rf build/ dist/ htmlcov/ .coverage

# Build the project
build: clean
	python setup.py build

# Docker commands
docker-build:
	docker build -t amr-system .

docker-run:
	docker-compose up -d

docker-stop:
	docker-compose down

# Development setup
dev-setup: install
	pre-commit install

# Production setup
prod-setup: install
	python -m config.system_config
