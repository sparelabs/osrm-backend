# OSRM Backend - macOS Build Instructions

This guide will help you reproduce the macOS builds for OSRM backend using a macOS virtual machine setup.

## Prerequisites

- Tart virtualization tool installed
- macOS Ventura image (as shown in your setup)

## Initial Setup

```bash
# Clone the macOS image
tart clone ghcr.io/cirruslabs/macos-ventura-xcode:latest osrm-ventura

# Start the VM
tart run osrm-ventura
```

## Inside the macOS VM

```bash
ssh admin@$(tart ip osrm-ventura) # admin:admin
```

### 1. Install Dependencies

```bash
# Install required tools (but not the default node)
brew install ccache cmake git wget pyenv

# Install Node.js 22 specifically (required version)
brew install node@22
brew link node@22 --force

echo 'export PATH="/opt/homebrew/opt/node@22/bin:$PATH"' >> ~/.zshrc


# Set up pyenv in your shell
echo 'export PYENV_ROOT="$HOME/.pyenv"' >> ~/.zshrc
echo 'command -v pyenv >/dev/null || export PATH="$PYENV_ROOT/bin:$PATH"' >> ~/.zshrc
echo 'eval "$(pyenv init -)"' >> ~/.zshrc

# Reload shell or source the file
source ~/.zshrc

# Install Python 3.10 via pyenv
pyenv install 3.10
pyenv global 3.10

# Install Conan package manager
pip install conan==1.61.0

# Verify versions
node --version  # Should show v22.x.x
python --version  # Should show Python 3.10.12
which python  # Should show ~/.pyenv/shims/python3
```

### 2. Clone OSRM Backend

```bash
git clone https://github.com/dehydr8/osrm-backend.git
cd osrm-backend
git checkout 5.27.1-spare.0
```

### 3. Install Threading Building Blocks (TBB)

**NOTE:** Not needed for Conan builds!

```bash
# Set TBB version
TBB_VERSION=2021.3.0
TBB_URL="https://github.com/oneapi-src/oneTBB/releases/download/v${TBB_VERSION}/oneapi-tbb-${TBB_VERSION}-mac.tgz"

# Download and install TBB
wget --tries 5 ${TBB_URL} -O onetbb.tgz
tar zxvf onetbb.tgz
sudo cp -a oneapi-tbb-${TBB_VERSION}/lib/. /usr/local/lib/
sudo cp -a oneapi-tbb-${TBB_VERSION}/include/. /usr/local/include/
```

### 4. Setup Environment Variables

```bash
# Set up build directories
export OSRM_INSTALL_DIR="${PWD}/install-osrm"
export OSRM_BUILD_DIR="${PWD}/build-osrm"

# Set up compiler
export CC=clang
export CXX=clang++
export CXXFLAGS="-Wno-unused-but-set-variable"

# Set number of parallel jobs
export JOBS=$((`sysctl -n hw.ncpu` + 1))

# Configure ccache
export CCACHE_TEMPDIR=/tmp/.ccache-temp
export CCACHE_COMPRESS=1
```

### 5. Prepare Build Environment

```bash
# Create build directory
mkdir -p ${OSRM_BUILD_DIR}

# Configure ccache
ccache --max-size=256M

# Install npm dependencies
npm ci --ignore-scripts
```

### 6. Build Configuration Options

#### Fix XCode Path

```bash
sudo xcode-select --switch /Library/Developer/CommandLineTools
export LIBRARY_PATH=${LIBRARY_PATH}:/Library/Developer/CommandLineTools/SDKs/MacOSX.sdk/usr/lib
export CPATH=${CPATH}:/Library/Developer/CommandLineTools/SDKs/MacOSX.sdk/usr/include
```

#### Standard Build (Intel/AMD64)

```bash
cd ${OSRM_BUILD_DIR}

cmake .. \
  -DCMAKE_BUILD_TYPE=Release \
  -DENABLE_CONAN=ON \
  -DENABLE_ASSERTIONS=ON \
  -DENABLE_NODE_BINDINGS=ON \
  -DBUILD_TOOLS=ON \
  -DENABLE_CCACHE=ON \
  -DCMAKE_INSTALL_PREFIX=${OSRM_INSTALL_DIR}
```

#### Apple Silicon (ARM64) Cross-Compilation Build

```bash
cd ${OSRM_BUILD_DIR}

# Set up Apple Silicon cross-compilation
ARCH=arm64
TARGET="${ARCH}-apple-darwin"
CFLAGS="--target=$TARGET"
CXXFLAGS="--target=$TARGET"

cmake .. \
  -DCMAKE_BUILD_TYPE=Release \
  -DENABLE_CONAN=ON \
  -DENABLE_ASSERTIONS=ON \
  -DENABLE_NODE_BINDINGS=ON \
  -DBUILD_TOOLS=ON \
  -DENABLE_CCACHE=ON \
  -DCMAKE_INSTALL_PREFIX=${OSRM_INSTALL_DIR} \
  -DCMAKE_C_COMPILER_TARGET="$TARGET" \
  -DCMAKE_CXX_COMPILER_TARGET="$TARGET" \
  -DCMAKE_SYSTEM_PROCESSOR="${ARCH}" \
  -DCMAKE_SYSTEM_NAME="Darwin" \
  -DCMAKE_C_FLAGS="$CFLAGS" \
  -DCMAKE_CXX_FLAGS="$CXXFLAGS"
```

### 7. Build OSRM

```bash
# Build the main project
make --jobs=${JOBS}

# Build tests (skip for Apple Silicon cross-compilation)
if [[ "$(uname -m)" == "arm64" ]] || [[ "${ENABLE_APPLE_SILICON}" != "ON" ]]; then
  make tests --jobs=${JOBS}
  make benchmarks --jobs=${JOBS}
fi

# Check ccache statistics
ccache -s

# Install OSRM
sudo make install
```

### 8. Build Example Project

```bash
# Only build example for native builds (not cross-compilation)
cd ${PWD}/example
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make --jobs=${JOBS}
```

### 9. Run Tests

```bash
# Prepare test data
make -C test/data benchmark

# Run the example
./example/build/osrm-example test/data/mld/monaco.osrm

# Run unit tests
cd ${OSRM_BUILD_DIR}
for i in ./unit_tests/*-tests; do
  echo "Running $i"
  $i
done

# Run Node.js tests
npm run nodejs-tests

# Run main test suite
cd ${PWD}
npm test
```

### 10. Verify Apple Silicon Build (if cross-compiling)

```bash
# Check if the Node.js binding was built for ARM64
ARCH=$(file ./lib/binding/node_osrm.node | awk '{printf $NF}')
echo "Architecture: $ARCH"

if [[ "$ARCH" == "arm64" ]]; then
  echo "✅ Successfully built for Apple Silicon"
else
  echo "❌ Build architecture mismatch"
  file ./lib/binding/node_osrm.node
fi
```

### 11. Publish!

Run the script to prepare the release package.

```bash
export BUILD_TYPE=Release
export ENABLE_APPLE_SILICON=OFF
./scripts/ci/node_package.sh

```

## Build Variants

### Debug Build

Replace `Release` with `Debug` in the cmake configuration:

```bash
-DCMAKE_BUILD_TYPE=Debug
```

### With Sanitizers

Add sanitizer flags:

```bash
-DENABLE_SANITIZER=ON
```

### Coverage Build

For code coverage analysis:

```bash
-DENABLE_COVERAGE=ON
# Also install: brew install lcov
```

## Troubleshooting

### "C compiler cannot create executables" Error

If you encounter the following error during Conan build:

```
configure: error: C compiler cannot create executables
ERROR: libiconv/1.17: Error in build() method
```

This is typically caused by missing or misconfigured development tools. Try these solutions in order:

#### Solution 1: Install/Reinstall Xcode Command Line Tools

```bash
# Remove existing installation
sudo rm -rf /Library/Developer/CommandLineTools

# Install fresh command line tools
xcode-select --install

# Accept license
sudo xcodebuild -license accept

# Verify installation
xcode-select -p
# Should output: /Library/Developer/CommandLineTools
```

#### Solution 2: Configure Conan Profile

```bash
# Detect and create default profile
conan profile detect --force

# Check the profile
conan profile show default

# If needed, manually set the profile
conan profile new default --detect --force
conan profile update settings.compiler=apple-clang default
conan profile update settings.compiler.version=14 default  # Adjust based on your Xcode version
conan profile update settings.compiler.libcxx=libstdc++11 default
```

#### Solution 3: Set Proper Environment Variables

```bash
# Set SDK path explicitly
export SDKROOT=$(xcrun --show-sdk-path)
export MACOSX_DEPLOYMENT_TARGET=13.0  # Adjust for your macOS version

# Verify SDK path
echo $SDKROOT
# Should output something like: /Applications/Xcode_14.3.1.app/Contents/Developer/Platforms/MacOSX.platform/Developer/SDKs/MacOSX.sdk
```

#### Solution 4: Clean Conan Cache

```bash
# Remove problematic package cache
conan remove libiconv/1.17 --force

# Clean all cache if needed
conan remove "*" --force

# Clear conan cache directory
rm -rf ~/.conan/data
```

#### Solution 5: Use System Dependencies Instead

If Conan continues to fail, disable Conan and use system dependencies:

```bash
# Install system dependencies via Homebrew
brew install libiconv boost lua

# Configure without Conan
cd ${OSRM_BUILD_DIR}
cmake .. \
  -DCMAKE_BUILD_TYPE=Release \
  -DENABLE_CONAN=OFF \
  -DENABLE_ASSERTIONS=ON \
  -DENABLE_NODE_BINDINGS=ON \
  -DBUILD_TOOLS=ON \
  -DENABLE_CCACHE=ON \
  -DCMAKE_INSTALL_PREFIX=${OSRM_INSTALL_DIR}
```

#### Solution 6: For Apple Silicon Cross-Compilation Issues

If you're cross-compiling for Apple Silicon, the issue might be with the target architecture:

```bash
# Ensure proper cross-compilation setup
export ARCH=arm64
export TARGET="${ARCH}-apple-darwin"

# Create a specific Conan profile for cross-compilation
conan profile new arm64 --detect
conan profile update settings.arch=armv8 arm64
conan profile update settings.os=Macos arm64

# Use the profile
conan install .. --profile arm64
```

### Common Issues

1. **TBB not found**: Ensure TBB is properly installed in `/usr/local/`
2. **Conan issues**: Try `conan profile detect --force` to regenerate profile
3. **Node.js version mismatch**: Ensure you have Node.js 22 (see Node.js version fix below)
4. **ccache not working**: Check `ccache -s` and verify paths

### Node.js Version Issues

If you have the wrong Node.js version installed:

```bash
# Check current version
node --version

# If you have the default version (20.x), switch to 22
brew unlink node
brew install node@22
brew link node@22 --force

# Or if you need to manage multiple versions, use n or nvm
npm install -g n
n 22

# Verify correct version
node --version  # Should show v22.x.x
```

### Python/pyenv Issues

If you encounter Python-related issues:

```bash
# Check if pyenv is properly set up
pyenv versions  # Should show * 3.10.12 (set by ~/.pyenv/version)

# If pyenv isn't working, ensure shell configuration is correct
# For bash users, replace ~/.zshrc with ~/.bash_profile or ~/.bashrc
echo 'export PYENV_ROOT="$HOME/.pyenv"' >> ~/.zshrc
echo 'command -v pyenv >/dev/null || export PATH="$PYENV_ROOT/bin:$PATH"' >> ~/.zshrc
echo 'eval "$(pyenv init -)"' >> ~/.zshrc

# Restart terminal or source the file
source ~/.zshrc

# Verify Python version and location
python3 --version  # Should show Python 3.10.12
which python3     # Should show ~/.pyenv/shims/python3

# If conan installation fails, try upgrading pip first
python3 -m pip install --upgrade pip
python3 -m pip install conan==1.61.0
```

### Environment Reset

```bash
# Clean build directory
rm -rf ${OSRM_BUILD_DIR}
mkdir -p ${OSRM_BUILD_DIR}

# Clear ccache
ccache -C

# Clean npm
npm clean-install

# Reset Python environment if needed
pyenv global 3.10.12
python3 -m pip install --upgrade pip
python3 -m pip install conan==1.61.0
```

## Performance Notes

- Use `ccache` for faster rebuilds
- Adjust `JOBS` based on your VM's CPU allocation
- Consider increasing VM RAM for parallel builds (8GB+ recommended)

## Additional Resources

- [OSRM Backend Documentation](https://github.com/Project-OSRM/osrm-backend)
- [Conan Documentation](https://docs.conan.io/)
- [CMake Documentation](https://cmake.org/documentation/)

### "Variable set but not used" Compilation Error

If you encounter compilation errors like:

```
error: variable 'current_level' set but not used [-Werror,-Wunused-but-set-variable]
```

This is caused by strict compiler warnings being treated as errors. Here are several solutions:

#### Solution 1: Disable the Specific Warning (Recommended)

```bash
cd ${OSRM_BUILD_DIR}
cmake .. \
  -DCMAKE_BUILD_TYPE=Release \
  -DENABLE_CONAN=ON \
  -DENABLE_ASSERTIONS=ON \
  -DENABLE_NODE_BINDINGS=ON \
  -DBUILD_TOOLS=ON \
  -DENABLE_CCACHE=ON \
  -DCMAKE_INSTALL_PREFIX=${OSRM_INSTALL_DIR} \
  -DCMAKE_CXX_FLAGS="-Wno-unused-but-set-variable"
```

#### Solution 2: Disable All Warnings as Errors

```bash
cd ${OSRM_BUILD_DIR}
cmake .. \
  -DCMAKE_BUILD_TYPE=Release \
  -DENABLE_CONAN=ON \
  -DENABLE_ASSERTIONS=ON \
  -DENABLE_NODE_BINDINGS=ON \
  -DBUILD_TOOLS=ON \
  -DENABLE_CCACHE=ON \
  -DCMAKE_INSTALL_PREFIX=${OSRM_INSTALL_DIR} \
  -DCMAKE_CXX_FLAGS="-Wno-error"
```

#### Solution 3: Fix the Code (Advanced)

If you want to fix the actual code issue, you can mark the variable as potentially unused:

```bash
# Edit the problematic file
cd ~/osrm-backend
sed -i '' 's/unsigned current_level = 0;/[[maybe_unused]] unsigned current_level = 0;/' src/contractor/graph_contractor.cpp
```

#### Solution 4: Use Debug Build

Debug builds often have relaxed warning settings:

```bash
cd ${OSRM_BUILD_DIR}
cmake .. \
  -DCMAKE_BUILD_TYPE=Debug \
  -DENABLE_CONAN=ON \
  -DENABLE_NODE_BINDINGS=ON \
  -DBUILD_TOOLS=ON \
  -DENABLE_CCACHE=ON \
  -DCMAKE_INSTALL_PREFIX=${OSRM_INSTALL_DIR}
```
