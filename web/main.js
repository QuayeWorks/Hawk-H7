const canvas = document.getElementById("renderCanvas");
const engine = new BABYLON.Engine(canvas, true, { stencil: true });

function createScene() {
  const scene = new BABYLON.Scene(engine);
  scene.clearColor = new BABYLON.Color4(0.02, 0.06, 0.09, 1.0);

  const camera = new BABYLON.ArcRotateCamera(
    "camera",
    BABYLON.Tools.ToRadians(140),
    BABYLON.Tools.ToRadians(65),
    35,
    new BABYLON.Vector3(0, 3, 0),
    scene
  );
  camera.attachControl(canvas, true);
  camera.lowerBetaLimit = BABYLON.Tools.ToRadians(15);
  camera.upperBetaLimit = BABYLON.Tools.ToRadians(80);
  camera.lowerRadiusLimit = 8;
  camera.upperRadiusLimit = 80;

  const sun = new BABYLON.DirectionalLight(
    "sun",
    new BABYLON.Vector3(-0.7, -1, 0.3),
    scene
  );
  sun.position = new BABYLON.Vector3(25, 40, -20);
  sun.intensity = 1.15;
  const hemispheric = new BABYLON.HemisphericLight(
    "hemi",
    new BABYLON.Vector3(0, 1, 0),
    scene
  );
  hemispheric.intensity = 0.35;

  const shadowGenerator = new BABYLON.ShadowGenerator(2048, sun);
  shadowGenerator.useBlurExponentialShadowMap = true;
  shadowGenerator.blurKernel = 32;
  shadowGenerator.darkness = 0.3;

  BABYLON.MeshBuilder.CreateGround(
    "ground",
    { width: 120, height: 120, subdivisions: 32 },
    scene
  );
  const groundMaterial = createGroundMaterial(scene);
  const ground = scene.getMeshByName("ground");
  ground.material = groundMaterial;
  ground.receiveShadows = true;

  const treeTemplate = buildProceduralTree(scene);
  shadowGenerator.addShadowCaster(treeTemplate);
  treeTemplate.setEnabled(false);

  scatterForest(scene, treeTemplate, shadowGenerator);
  scene.onAnimationGroupAddedObservable.add((animationGroup) => {
    if (animationGroup.name.toLowerCase().includes("punch")) {
      correctPunchOrientation(animationGroup);
    }
  });
  scene.animationGroups.forEach((animationGroup) => {
    if (animationGroup.name.toLowerCase().includes("punch")) {
      correctPunchOrientation(animationGroup);
    }
  });
  addPostProcess(scene, camera);
  return scene;
}

function correctPunchOrientation(animationGroup) {
  const armKeywords = ["arm", "forearm", "hand"];

  const flipQuaternion = (value) => {
    let quaternion;
    if (value instanceof BABYLON.Quaternion) {
      quaternion = value.clone();
    } else if (Array.isArray(value)) {
      quaternion = BABYLON.Quaternion.FromArray(value);
    } else if (
      value &&
      typeof value === "object" &&
      "x" in value &&
      "y" in value &&
      "z" in value &&
      "w" in value
    ) {
      quaternion = new BABYLON.Quaternion(value.x, value.y, value.z, value.w);
    } else {
      quaternion = BABYLON.Quaternion.Identity();
    }

    quaternion.x *= -1;
    quaternion.z *= -1;
    return quaternion;
  };

  const flipEuler = (value) => {
    if (value instanceof BABYLON.Vector3) {
      return new BABYLON.Vector3(value.x, -value.y, value.z);
    }
    if (Array.isArray(value)) {
      return new BABYLON.Vector3(value[0], -value[1], value[2]);
    }
    if (value && typeof value === "object" && "x" in value && "y" in value && "z" in value) {
      return new BABYLON.Vector3(value.x, -value.y, value.z);
    }
    return BABYLON.Vector3.Zero();
  };

  animationGroup.targetedAnimations.forEach(({ target, animation }) => {
    if (!target || !target.name) {
      return;
    }
    const targetName = target.name.toLowerCase();
    const matchesArm = armKeywords.some((keyword) => targetName.includes(keyword));
    if (!matchesArm) {
      return;
    }

    const keys = animation.getKeys().map((key) => {
      const cloned = { ...key };
      if (animation.targetProperty === "rotationQuaternion") {
        cloned.value = flipQuaternion(key.value);
      } else if (animation.targetProperty === "rotation") {
        cloned.value = flipEuler(key.value);
      }
      return cloned;
    });

    animation.setKeys(keys);
  });
}

function createGroundMaterial(scene) {
  const groundMaterial = new BABYLON.StandardMaterial("groundMat", scene);
  groundMaterial.specularColor = new BABYLON.Color3(0.05, 0.05, 0.05);
  groundMaterial.specularPower = 32;

  const textureSize = 1024;
  const groundTexture = new BABYLON.DynamicTexture(
    "groundTexture",
    { width: textureSize, height: textureSize },
    scene,
    false
  );
  const context = groundTexture.getContext();
  const { width, height } = groundTexture.getSize();

  const gradient = context.createRadialGradient(
    width / 2,
    height / 2,
    width * 0.15,
    width / 2,
    height / 2,
    width * 0.7
  );
  gradient.addColorStop(0, "#3f8c44");
  gradient.addColorStop(0.35, "#3b7c3c");
  gradient.addColorStop(0.65, "#356236");
  gradient.addColorStop(1, "#4b3a25");
  context.fillStyle = gradient;
  context.fillRect(0, 0, width, height);

  const imageData = context.getImageData(0, 0, width, height);
  const data = imageData.data;
  const noiseStrength = 18;
  for (let i = 0; i < data.length; i += 4) {
    const noise = (Math.random() - 0.5) * noiseStrength;
    data[i] = Math.max(0, Math.min(255, data[i] + noise));
    data[i + 1] = Math.max(0, Math.min(255, data[i + 1] + noise * 0.6));
    data[i + 2] = Math.max(0, Math.min(255, data[i + 2] + noise * 0.4));
  }
  context.putImageData(imageData, 0, 0);
  groundTexture.update(false);
  groundTexture.wrapU = BABYLON.Texture.WRAP_ADDRESSING_MODE;
  groundTexture.wrapV = BABYLON.Texture.WRAP_ADDRESSING_MODE;
  groundTexture.uScale = 2;
  groundTexture.vScale = 2;

  groundMaterial.diffuseTexture = groundTexture;
  groundMaterial.diffuseColor = new BABYLON.Color3(0.35, 0.52, 0.32);
  groundMaterial.ambientColor = new BABYLON.Color3(0.3, 0.38, 0.25);

  const roughnessTexture = new BABYLON.NoiseProceduralTexture(
    "groundNoise",
    256,
    scene
  );
  roughnessTexture.animationSpeedFactor = 0.15;
  roughnessTexture.persistence = 2;
  roughnessTexture.brightness = 0.3;
  roughnessTexture.octaves = 3;
  groundMaterial.bumpTexture = roughnessTexture;
  groundMaterial.bumpTexture.level = 0.6;

  return groundMaterial;
}

function buildProceduralTree(scene) {
  const trunkHeight = 4;
  const trunkDiameter = 0.8;
  const trunk = BABYLON.MeshBuilder.CreateCylinder(
    "trunk",
    {
      diameter: trunkDiameter,
      height: trunkHeight,
      tessellation: 8,
      subdivisions: 1,
      enclose: true,
    },
    scene
  );
  trunk.position.y = trunkHeight / 2;

  const trunkMaterial = new BABYLON.StandardMaterial("trunkMat", scene);
  trunkMaterial.diffuseColor = new BABYLON.Color3(0.35, 0.2, 0.1);
  trunkMaterial.bumpTexture = new BABYLON.NoiseProceduralTexture(
    "trunkNoise",
    256,
    scene
  );
  trunkMaterial.bumpTexture.animationSpeedFactor = 0.5;
  trunk.material = trunkMaterial;

  const foliage = BABYLON.MeshBuilder.CreateSphere(
    "foliage",
    { diameterX: 5, diameterY: 3.4, diameterZ: 5, segments: 8 },
    scene
  );
  foliage.position.y = trunkHeight;

  const foliageMaterial = new BABYLON.StandardMaterial("foliageMat", scene);
  foliageMaterial.diffuseColor = new BABYLON.Color3(0.09, 0.35, 0.12);
  foliageMaterial.specularColor = new BABYLON.Color3(0.05, 0.1, 0.05);
  foliage.material = foliageMaterial;

  const tree = BABYLON.Mesh.MergeMeshes([trunk, foliage], true, false, undefined, false, true);
  tree.name = "tree";

  applyWindAnimation(tree, scene);
  return tree;
}

function applyWindAnimation(tree, scene) {
  const pivot = new BABYLON.TransformNode(`${tree.name}-pivot`, scene);
  tree.parent = pivot;
  pivot.position = tree.position.clone();

  const anim = new BABYLON.Animation(
    "wind",
    "rotation.z",
    30,
    BABYLON.Animation.ANIMATIONTYPE_FLOAT,
    BABYLON.Animation.ANIMATIONLOOPMODE_CYCLE
  );

  const keys = [
    { frame: 0, value: BABYLON.Tools.ToRadians(-1.8) },
    { frame: 40, value: BABYLON.Tools.ToRadians(1.8) },
    { frame: 80, value: BABYLON.Tools.ToRadians(-1.3) },
  ];
  anim.setKeys(keys);
  anim.enableBlending = true;
  anim.blendingSpeed = 0.02;

  pivot.animations.push(anim);
  scene.beginAnimation(pivot, 0, 80, true);
}

function scatterForest(scene, template, shadowGenerator) {
  const instances = [];
  const treeCount = 150;
  for (let i = 0; i < treeCount; i += 1) {
    const treeInstance = template.clone(`tree-${i}`);
    treeInstance.setEnabled(true);

    const radius = 20 + Math.random() * 35;
    const angle = Math.random() * Math.PI * 2;
    treeInstance.position = new BABYLON.Vector3(
      Math.cos(angle) * radius,
      0,
      Math.sin(angle) * radius
    );

    const scaleY = 0.8 + Math.random() * 0.9;
    const scaleXZ = 0.6 + Math.random() * 0.8;
    treeInstance.scaling = new BABYLON.Vector3(scaleXZ, scaleY, scaleXZ);
    treeInstance.rotation.y = Math.random() * Math.PI * 2;

    instances.push(treeInstance);
    shadowGenerator.addShadowCaster(treeInstance);
  }

  const clearingRadius = 12;
  instances.forEach((tree) => {
    const distance = Math.sqrt(tree.position.x ** 2 + tree.position.z ** 2);
    if (distance < clearingRadius) {
      tree.position.x = (tree.position.x / distance) * clearingRadius;
      tree.position.z = (tree.position.z / distance) * clearingRadius;
    }
  });
}

function addPostProcess(scene, camera) {
  const pipeline = new BABYLON.DefaultRenderingPipeline(
    "defaultPipeline",
    true,
    scene,
    [camera]
  );
  pipeline.imageProcessingEnabled = true;
  pipeline.imageProcessing.contrast = 1.2;
  pipeline.imageProcessing.exposure = 1.15;
  pipeline.bloomEnabled = true;
  pipeline.bloomThreshold = 0.7;
  pipeline.bloomWeight = 0.25;
  pipeline.bloomKernel = 24;
  pipeline.fxaaEnabled = true;
}

const scene = createScene();
engine.runRenderLoop(() => {
  scene.render();
});

window.addEventListener("resize", () => {
  engine.resize();
});
