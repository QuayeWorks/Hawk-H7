const DEFAULT_GROUND_SAMPLE_INTERVAL = 5;

/**
 * Updates the desired vertical offset that keeps the player's root aligned
 * with the sampled ground height.
 *
 * @param {Object} state - Runtime locomotion state that stores the ground
 *   alignment parameters.
 * @param {BABYLON.TransformNode} playerRoot - Root transform for the
 *   character that is being aligned.
 * @param {(root: BABYLON.TransformNode) => number} sampleRootGroundOffset -
 *   Callback that returns the delta between the desired ground height and the
 *   current foot bottom position (desired - bottom).
 */
export function updateRootGroundOffset(
  state,
  playerRoot,
  sampleRootGroundOffset
) {
  if (!state.grounded) {
    return;
  }

  if (state.groundSampleCountdown > 0) {
    state.groundSampleCountdown -= 1;
  }

  if (!state.groundSampleDirty && state.groundSampleCountdown > 0) {
    return;
  }

  playerRoot.computeWorldMatrix(true);
  const sampleOffset = sampleRootGroundOffset(playerRoot);
  if (Number.isFinite(sampleOffset)) {
    const currentOffset = state.rootGroundOffset ?? 0;
    state.rootGroundOffsetTarget = currentOffset + sampleOffset;
  }

  state.groundSampleDirty = false;
  state.groundSampleCountdown =
    state.groundSampleInterval ?? DEFAULT_GROUND_SAMPLE_INTERVAL;
}

/**
 * Smooths the vertical offset that is applied to the player's root so the
 * adjustment converges gradually toward the sampled target.
 *
 * @param {Object} state - Locomotion state containing the offsets.
 */
export function applyRootGroundOffsetSmoothing(state) {
  const lerp = state.rootGroundOffsetLerp ?? 0;
  if (lerp <= 0) {
    return;
  }

  const targetDelta = state.rootGroundOffsetTarget - state.rootGroundOffset;
  state.rootGroundOffset += targetDelta * lerp;
}

export const GameMode = Object.freeze({
  MENU: "menu",
  GAME: "game",
  CREATOR: "creator",
  RIG: "rig",
});

function collectSceneCounts(scene) {
  if (!scene || scene.isDisposed()) {
    return { nodes: 0, materials: 0, textures: 0 };
  }

  const nodes = scene.getNodes ? scene.getNodes().length : 0;
  const materials = scene.materials?.length ?? 0;
  const textures = scene.textures?.length ?? 0;

  return { nodes, materials, textures };
}

class MenuBackground {
  /**
   * @param {BABYLON.Scene} scene
   */
  constructor(scene) {
    this.scene = scene;
    this.root = new BABYLON.TransformNode("menuBackgroundRoot", scene);

    this.light = new BABYLON.HemisphericLight(
      "menuBackgroundLight",
      new BABYLON.Vector3(0.15, 1, 0.25),
      scene
    );
    this.light.intensity = 0.8;
    this.light.parent = this.root;

    this.mesh = BABYLON.MeshBuilder.CreateSphere(
      "menuBackgroundSky",
      { diameter: 50, sideOrientation: BABYLON.Mesh.BACKSIDE },
      scene
    );
    this.mesh.parent = this.root;

    this.material = new BABYLON.StandardMaterial(
      "menuBackgroundMaterial",
      scene
    );
    this.material.backFaceCulling = false;
    this.material.disableLighting = true;

    this.texture = new BABYLON.DynamicTexture(
      "menuBackgroundTexture",
      { width: 512, height: 512 },
      scene,
      false
    );
    this.texture.hasAlpha = false;

    const ctx = this.texture.getContext();
    const gradient = ctx.createLinearGradient(0, 0, 0, 512);
    gradient.addColorStop(0, "#1f3e86");
    gradient.addColorStop(0.45, "#0f1d3c");
    gradient.addColorStop(1, "#040810");
    ctx.fillStyle = gradient;
    ctx.fillRect(0, 0, 512, 512);

    ctx.font = "48px 'Segoe UI', sans-serif";
    ctx.fillStyle = "rgba(255, 255, 255, 0.12)";
    ctx.fillText("Hawk-H7 Systems", 40, 120);
    ctx.font = "22px 'Segoe UI', sans-serif";
    ctx.fillStyle = "rgba(255, 255, 255, 0.25)";
    ctx.fillText("Menu Background", 40, 170);
    this.texture.update();

    this.material.diffuseTexture = this.texture;
    this.mesh.material = this.material;

    this._rotationObserver = scene.onBeforeRenderObservable.add(() => {
      if (!this.root || this.root.isDisposed()) {
        return;
      }

      const delta = scene.getEngine().getDeltaTime();
      const radians = (delta / 1000) * Math.PI * 0.025;
      this.root.rotate(BABYLON.Axis.Y, radians, BABYLON.Space.WORLD);
    });

    this._pulseInterval = window.setInterval(() => {
      if (!this.light || this.light.isDisposed()) {
        return;
      }
      this.light.intensity = 0.65 + Math.random() * 0.25;
    }, 1600);

    this._domAnchor =
      document.getElementById("render-root") ?? document.body;
    this._domRoot = document.createElement("div");
    this._domRoot.className = "menu-decor";
    this._domRoot.textContent = "Menu background active";
    this._domAnchor.appendChild(this._domRoot);

    this._sceneDisposeObserver = scene.onDisposeObservable.add(() => {
      this.dispose();
    });

    this._disposed = false;
  }

  dispose() {
    if (this._disposed) {
      return;
    }
    this._disposed = true;

    if (this._sceneDisposeObserver) {
      this.scene?.onDisposeObservable.remove(this._sceneDisposeObserver);
      this._sceneDisposeObserver = null;
    }

    if (this._rotationObserver) {
      this.scene?.onBeforeRenderObservable.remove(this._rotationObserver);
      this._rotationObserver = null;
    }

    if (this._pulseInterval != null) {
      window.clearInterval(this._pulseInterval);
      this._pulseInterval = null;
    }

    this._domRoot?.remove();
    this._domRoot = null;
    this._domAnchor = null;

    this.texture?.dispose();
    this.texture = null;

    this.material?.dispose();
    this.material = null;

    this.mesh?.dispose();
    this.mesh = null;

    this.light?.dispose();
    this.light = null;

    this.root?.dispose();
    this.root = null;

    this.scene = null;
  }
}

class ExperienceController {
  /**
   * @param {HTMLCanvasElement} canvas
   * @param {{ recordTransition: Function } | null} auditPanel
   */
  constructor(canvas, auditPanel) {
    if (!canvas) {
      throw new Error("A canvas is required to initialize the experience");
    }

    this.canvas = canvas;
    this.engine = new BABYLON.Engine(canvas, true, {
      preserveDrawingBuffer: true,
      stencil: true,
    });
    this.auditPanel = auditPanel ?? null;
    this.currentMode = null;
    this.scene = null;
    this.menuBackground = null;

    this._renderLoop = this._renderLoop.bind(this);
    this.engine.runRenderLoop(this._renderLoop);

    this._resizeHandler = () => {
      this.engine.resize();
    };
    window.addEventListener("resize", this._resizeHandler);
  }

  _renderLoop() {
    if (this.scene && !this.scene.isDisposed()) {
      this.scene.render();
    }
  }

  _createBaseScene() {
    const scene = new BABYLON.Scene(this.engine);
    scene.clearColor = new BABYLON.Color4(0.025, 0.032, 0.07, 1);

    const camera = new BABYLON.ArcRotateCamera(
      "camera",
      Math.PI / 2,
      Math.PI / 2.2,
      8,
      BABYLON.Vector3.Zero(),
      scene
    );
    camera.attachControl(this.canvas, true);
    camera.lowerRadiusLimit = 4;
    camera.upperRadiusLimit = 20;

    new BABYLON.HemisphericLight(
      "sceneLight",
      new BABYLON.Vector3(0.1, 1, 0.3),
      scene
    ).intensity = 0.85;

    const ground = BABYLON.MeshBuilder.CreateGround(
      "ground",
      { width: 18, height: 18 },
      scene
    );
    const groundMaterial = new BABYLON.StandardMaterial("groundMaterial", scene);
    groundMaterial.diffuseColor = new BABYLON.Color3(0.1, 0.16, 0.22);
    groundMaterial.specularColor = new BABYLON.Color3(0, 0, 0);
    ground.material = groundMaterial;

    return scene;
  }

  _populateModeContent(scene, mode) {
    const colorMap = {
      [GameMode.MENU]: new BABYLON.Color3(0.45, 0.65, 1),
      [GameMode.GAME]: new BABYLON.Color3(0.6, 0.86, 0.4),
      [GameMode.CREATOR]: new BABYLON.Color3(0.94, 0.74, 0.4),
      [GameMode.RIG]: new BABYLON.Color3(0.94, 0.45, 0.55),
    };

    const mesh = BABYLON.MeshBuilder.CreateBox(
      `${mode}Marker`,
      { size: 1.2 },
      scene
    );
    mesh.position.y = 1.1;
    mesh.material = new BABYLON.StandardMaterial(`${mode}Material`, scene);
    mesh.material.diffuseColor = colorMap[mode] ?? BABYLON.Color3.White();

    const label = BABYLON.GUI.AdvancedDynamicTexture.CreateFullscreenUI(
      `${mode}LabelUI`,
      true,
      scene
    );

    const panel = new BABYLON.GUI.StackPanel();
    panel.isVertical = true;
    panel.width = "220px";
    panel.horizontalAlignment = BABYLON.GUI.Control.HORIZONTAL_ALIGNMENT_RIGHT;
    panel.verticalAlignment = BABYLON.GUI.Control.VERTICAL_ALIGNMENT_TOP;
    panel.paddingRight = "24px";
    panel.paddingTop = "24px";

    const textBlock = new BABYLON.GUI.TextBlock();
    textBlock.text = `${mode.toUpperCase()} MODE`;
    textBlock.color = "#dde7ff";
    textBlock.fontSize = 18;
    textBlock.fontStyle = "bold";

    panel.addControl(textBlock);
    label.addControl(panel);

    return {
      mesh,
      label,
    };
  }

  setMode(mode) {
    if (!Object.values(GameMode).includes(mode)) {
      throw new Error(`Unknown mode: ${mode}`);
    }

    if (this.currentMode === mode) {
      return;
    }

    const previousScene = this.scene;
    const preCounts = collectSceneCounts(previousScene);

    if (this.menuBackground) {
      this.menuBackground.dispose();
      this.menuBackground = null;
    }

    if (previousScene && !previousScene.isDisposed()) {
      previousScene.dispose();
    }

    const scene = this._createBaseScene();
    const { label } = this._populateModeContent(scene, mode);

    if (mode === GameMode.MENU) {
      this.menuBackground = new MenuBackground(scene);
    }

    this.scene = scene;

    const finalizeRecord = () => {
      const postCounts = collectSceneCounts(scene);
      this.auditPanel?.recordTransition(this.currentMode, mode, preCounts, postCounts);
    };

    if (scene.isReady()) {
      finalizeRecord();
    } else {
      scene.executeWhenReady(finalizeRecord);
    }

    this.currentMode = mode;

    scene.onDisposeObservable.add(() => {
      label.dispose();
    });
  }

  dispose() {
    if (this.menuBackground) {
      this.menuBackground.dispose();
      this.menuBackground = null;
    }

    if (this.scene && !this.scene.isDisposed()) {
      this.scene.dispose();
    }

    this.engine.stopRenderLoop(this._renderLoop);
    this.engine.dispose();
    window.removeEventListener("resize", this._resizeHandler);
  }
}

/**
 * Bootstraps the Babylon.js experience and returns a controller that can
 * change modes.
 *
 * @param {HTMLCanvasElement} canvas
 * @param {{ recordTransition: Function } | null} auditPanel
 */
export function initializeExperience(canvas, auditPanel = null) {
  const controller = new ExperienceController(canvas, auditPanel);
  controller.setMode(GameMode.MENU);
  return controller;
}
