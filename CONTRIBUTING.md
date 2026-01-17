# Contributing to AURA

Thank you for your interest in contributing to AURA! This document provides guidelines for contributions.

## Getting Started

1. Fork the repository
2. Clone your fork: `git clone https://github.com/YOUR_USERNAME/aura-sensor-fusion.git`
3. Create a feature branch: `git checkout -b feature/your-feature-name`
4. Make your changes
5. Run tests: `ctest --test-dir build`
6. Submit a pull request

## Code Style

### Formatting

- Use the provided `.clang-format` configuration
- Run `clang-format -i <file>` before committing
- Maximum line length: 100 characters

### Naming Conventions

- Classes: `CamelCase`
- Functions: `camelBack`
- Variables: `snake_case`
- Constants: `kCamelCase`
- Private members: `snake_case_` (trailing underscore)

### C++ Guidelines

- Use C++20 features where appropriate
- Prefer `[[nodiscard]]` for functions with important return values
- Use `constexpr` for compile-time evaluable functions
- Document public APIs with Doxygen comments

## Pull Request Process

1. Update documentation for any new features
2. Add tests for new functionality
3. Ensure all tests pass
4. Update the changelog if applicable
5. Request review from maintainers

## Reporting Issues

When reporting bugs, please include:

- Operating system and compiler version
- Steps to reproduce
- Expected vs actual behavior
- Relevant log output

## License

By contributing, you agree that your contributions will be licensed under the MIT License.
