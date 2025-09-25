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
  const groundMaterial = new BABYLON.StandardMaterial("groundMat", scene);
  groundMaterial.diffuseColor = new BABYLON.Color3(0.09, 0.24, 0.12);
  groundMaterial.specularColor = BABYLON.Color3.Black();
  const ground = scene.getMeshByName("ground");
  ground.material = groundMaterial;
  ground.receiveShadows = true;

  const treeTemplate = buildProceduralTree(scene);
  shadowGenerator.addShadowCaster(treeTemplate);
  treeTemplate.setEnabled(false);

  scatterForest(scene, treeTemplate, shadowGenerator);
  addPostProcess(scene, camera);
  return scene;
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
