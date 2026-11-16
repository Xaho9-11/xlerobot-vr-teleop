#!/usr/bin/env bash
# Download the browser libraries of the digital twin at the exact versions the console was built with
# (three.js r160, urdf-loader 0.12.3) into this folder. They are not stored in the repository.
set -euo pipefail
cd "$(dirname "$0")"
CDN=https://cdn.jsdelivr.net/npm
get() { mkdir -p "$(dirname "$2")"; curl -fsSL "$CDN/$1" -o "$2"; echo "  $2"; }
get three@0.160.0/build/three.module.js                         three/three.module.js
get three@0.160.0/examples/jsm/controls/OrbitControls.js        three/jsm/controls/OrbitControls.js
get three@0.160.0/examples/jsm/environments/RoomEnvironment.js  three/jsm/environments/RoomEnvironment.js
get three@0.160.0/examples/jsm/loaders/ColladaLoader.js         three/jsm/loaders/ColladaLoader.js
get three@0.160.0/examples/jsm/loaders/STLLoader.js             three/jsm/loaders/STLLoader.js
get three@0.160.0/examples/jsm/loaders/TGALoader.js             three/jsm/loaders/TGALoader.js
get urdf-loader@0.12.3/src/URDFLoader.js                        urdf-loader/URDFLoader.js
get urdf-loader@0.12.3/src/URDFClasses.js                       urdf-loader/URDFClasses.js
echo "done"
