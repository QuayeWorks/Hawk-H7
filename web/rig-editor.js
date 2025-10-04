const SECRET_PHRASE = "QuayeWorks";
function formatTimestamp(date = new Date()) {
  const hours = date.getHours().toString().padStart(2, "0");
  const minutes = date.getMinutes().toString().padStart(2, "0");
  const seconds = date.getSeconds().toString().padStart(2, "0");
  const millis = date.getMilliseconds().toString().padStart(3, "0");
  return `${hours}:${minutes}:${seconds}.${millis}`;
}

function nextFrame() {
  return new Promise((resolve) => {
    requestAnimationFrame(() => resolve());
  });
}

async function waitSceneReady(scene) {
  if (!scene || typeof scene.isReady !== "function") {
    return;
  }

  if (scene.isReady()) {
    return;
  }

  if (typeof scene.whenReadyAsync === "function") {
    await scene.whenReadyAsync();
    return;
  }

  await new Promise((resolve) => {
    const disposeObserver = scene.onDisposeObservable?.add(() => {
      if (disposeObserver && scene.onDisposeObservable) {
        scene.onDisposeObservable.remove(disposeObserver);
      }
      resolve();
    });

    scene.executeWhenReady?.(() => {
      if (disposeObserver && scene.onDisposeObservable) {
        scene.onDisposeObservable.remove(disposeObserver);
      }
      resolve();
    });
  });
}

function createToastHost(host) {
  const container = document.createElement("div");
  container.className = "rig-editor__toast-host";
  host.appendChild(container);
  return container;
}

function showToast(toastHost, message, type = "error") {
  if (!toastHost) {
    console.warn("RigEditor toast host missing", message);
    return;
  }

  const toast = document.createElement("div");
  toast.className = `rig-editor__toast rig-editor__toast--${type}`;
  toast.textContent = message;
  toastHost.appendChild(toast);

  requestAnimationFrame(() => {
    toast.classList.add("is-visible");
  });

  window.setTimeout(() => {
    toast.classList.remove("is-visible");
    window.setTimeout(() => {
      toast.remove();
    }, 320);
  }, 4200);
}

class RigEditor {
  /**
   * @param {BABYLON.Scene | null} scene
   * @param {{ host?: HTMLElement | null }} options
   */
  constructor(scene, options = {}) {
    this.scene = scene ?? null;
    this.host = options.host ?? document.body;

    this._disposed = false;
    this._booting = false;
    this._gateUnlocked = false;
    this._babylonBindings = [];
    this._sceneDisposeObserver = null;

    this.root = document.createElement("section");
    this.root.className = "rig-editor";
    this.host?.appendChild(this.root);

    this.toastHost = createToastHost(this.root);

    this._buildLayout();

    if (this.scene?.onDisposeObservable) {
      this._sceneDisposeObserver = this.scene.onDisposeObservable.add(() => {
        this.dispose();
      });
    }
  }

  _buildLayout() {
    this.bootPanel = document.createElement("div");
    this.bootPanel.className = "rig-editor__boot";

    const gateTitle = document.createElement("h2");
    gateTitle.textContent = "Rig Editor Gate";
    this.bootPanel.appendChild(gateTitle);

    const gateDescription = document.createElement("p");
    gateDescription.className = "rig-editor__description";
    gateDescription.textContent = "Enter the access phrase to continue.";
    this.bootPanel.appendChild(gateDescription);

    this.gateForm = document.createElement("form");
    this.gateForm.className = "rig-editor__gate-form";

    this.gateInput = document.createElement("input");
    this.gateInput.type = "password";
    this.gateInput.required = true;
    this.gateInput.placeholder = "Access phrase";
    this.gateInput.autocomplete = "off";
    this.gateInput.spellcheck = false;
    this.gateInput.className = "rig-editor__gate-input";

    this.gateSubmit = document.createElement("button");
    this.gateSubmit.type = "submit";
    this.gateSubmit.textContent = "Unlock";
    this.gateSubmit.className = "rig-editor__gate-submit";

    this.gateForm.appendChild(this.gateInput);
    this.gateForm.appendChild(this.gateSubmit);
    this.bootPanel.appendChild(this.gateForm);

    this.logList = document.createElement("div");
    this.logList.className = "rig-editor__log";
    this.bootPanel.appendChild(this.logList);

    this.root.appendChild(this.bootPanel);

    this.workspace = document.createElement("div");
    this.workspace.className = "rig-editor__workspace";

    const workspaceHeader = document.createElement("header");
    workspaceHeader.className = "rig-editor__workspace-header";
    const workspaceTitle = document.createElement("h2");
    workspaceTitle.textContent = "Rig Editor";
    workspaceHeader.appendChild(workspaceTitle);

    this.testButton = document.createElement("button");
    this.testButton.type = "button";
    this.testButton.textContent = "Create Test Animation";
    this.testButton.className = "rig-editor__action";
    workspaceHeader.appendChild(this.testButton);

    this.workspace.appendChild(workspaceHeader);

    this.timeline = document.createElement("section");
    this.timeline.className = "rig-editor__panel rig-editor__panel--timeline";
    const timelineHeader = document.createElement("h3");
    timelineHeader.textContent = "Timeline";
    this.timeline.appendChild(timelineHeader);
    this.timelineContent = document.createElement("div");
    this.timelineContent.className = "rig-editor__panel-content";
    this.timeline.appendChild(this.timelineContent);

    this.dopeSheet = document.createElement("section");
    this.dopeSheet.className = "rig-editor__panel rig-editor__panel--dopesheet";
    const dopeHeader = document.createElement("h3");
    dopeHeader.textContent = "Dope Sheet";
    this.dopeSheet.appendChild(dopeHeader);
    this.dopeSheetContent = document.createElement("div");
    this.dopeSheetContent.className = "rig-editor__panel-content";
    this.dopeSheet.appendChild(this.dopeSheetContent);

    this.workspace.appendChild(this.timeline);
    this.workspace.appendChild(this.dopeSheet);

    this.root.appendChild(this.workspace);

    this._gateHandler = (event) => {
      event.preventDefault();
      if (this._booting || this._disposed) {
        return;
      }

      const value = (this.gateInput?.value ?? "").trim();
      if (value !== SECRET_PHRASE) {
        this._logPhase("gate", "Access denied", "error");
        this.gateInput?.classList.add("is-invalid");
        showToast(this.toastHost, "Gate remains locked.");
        window.setTimeout(() => {
          this.gateInput?.classList.remove("is-invalid");
          this.gateInput?.focus();
          if (this.gateInput) {
            this.gateInput.value = "";
          }
        }, 240);
        return;
      }

      this._logPhase("gate", "Access granted", "success");
      this._beginBoot();
    };

    this._testButtonHandler = () => {
      if (this._disposed) {
        return;
      }
      const timestamp = formatTimestamp();
      const message = `Test animation created @ ${timestamp}`;
      const item = document.createElement("div");
      item.className = "rig-editor__timeline-event";
      item.textContent = message;
      this.timelineContent?.appendChild(item);
      this.timelineContent.scrollTop = this.timelineContent.scrollHeight;
      this._logPhase("timeline", "Inserted mock keyframe", "success");
      showToast(this.toastHost, "Test animation recorded", "success");
    };

    this.gateForm?.addEventListener("submit", this._gateHandler);
    this.testButton?.addEventListener("click", this._testButtonHandler);

    window.setTimeout(() => {
      if (!this._disposed) {
        this.gateInput?.focus();
      }
    }, 0);
  }

  _setBootState(isBooting) {
    this._booting = isBooting;
    if (!this.gateInput || !this.gateSubmit) {
      return;
    }

    this.gateInput.disabled = isBooting;
    this.gateSubmit.disabled = isBooting;
    if (isBooting) {
      this.gateSubmit.textContent = "Booting...";
    } else {
      this.gateSubmit.textContent = "Unlock";
      if (!this._gateUnlocked) {
        this.gateInput.focus();
      }
    }
  }

  async _beginBoot() {
    this._setBootState(true);
    this._gateUnlocked = true;

    const steps = [
      {
        name: "gate",
        run: async () => {
          await nextFrame();
        },
      },
      {
        name: "rig load",
        run: async () => {
          await nextFrame();
          this._rigDefinition = {
            name: "Demo Rig",
            bones: ["root", "spine", "head"],
          };
          const info = document.createElement("div");
          info.className = "rig-editor__timeline-event";
          info.textContent = `${this._rigDefinition.name} loaded (${this._rigDefinition.bones.length} bones)`;
          this.timelineContent?.appendChild(info);
        },
      },
      {
        name: "timeline",
        run: async () => {
          await nextFrame();
          if (this.timelineContent) {
            this.timelineContent.innerHTML = "";
            const placeholder = document.createElement("div");
            placeholder.className = "rig-editor__timeline-event";
            placeholder.textContent = "Timeline ready";
            this.timelineContent.appendChild(placeholder);
          }
        },
      },
      {
        name: "dope sheet",
        run: async () => {
          await nextFrame();
          if (this.dopeSheetContent) {
            this.dopeSheetContent.innerHTML = "";
            const placeholder = document.createElement("div");
            placeholder.className = "rig-editor__dopesheet-row";
            placeholder.textContent = "Dope sheet ready";
            this.dopeSheetContent.appendChild(placeholder);
          }
        },
      },
      {
        name: "Babylon binding",
        run: async () => {
          await waitSceneReady(this.scene);
          await nextFrame();
          this._bindPreview();
        },
      },
    ];

    try {
      for (const step of steps) {
        if (this._disposed) {
          return;
        }
        this._logPhase(step.name, "start", "info");
        await step.run();
        if (this._disposed) {
          return;
        }
        this._logPhase(step.name, "complete", "success");
      }

      this._completeBoot();
    } catch (error) {
      console.error("RigEditor boot failed", error);
      this._logPhase("boot", error?.message ?? "Unknown failure", "error");
      showToast(this.toastHost, "Rig editor failed to start. Check console for details.");
      this._setBootState(false);
      this._gateUnlocked = false;
      this.gateInput?.focus();
    }
  }

  _bindPreview() {
    if (!this.scene || typeof BABYLON === "undefined") {
      return;
    }

    const { TransformNode, MeshBuilder, Color3 } = BABYLON;
    if (!TransformNode || !MeshBuilder) {
      return;
    }

    const root = new TransformNode("rigEditorPreviewRoot", this.scene);
    const preview = MeshBuilder.CreateBox(
      "rigEditorPreview",
      { size: 0.75 },
      this.scene
    );
    preview.position.y = 1.25;
    preview.parent = root;

    if (preview.material && "diffuseColor" in preview.material) {
      preview.material.diffuseColor = new Color3(0.92, 0.55, 0.75);
    } else {
      const material = new BABYLON.StandardMaterial("rigEditorPreviewMaterial", this.scene);
      material.diffuseColor = new Color3(0.92, 0.55, 0.75);
      preview.material = material;
    }

    this._babylonBindings.push(() => {
      preview.dispose();
      if (root) {
        root.dispose();
      }
    });
  }

  _completeBoot() {
    this._setBootState(false);
    this.bootPanel?.classList.add("is-hidden");
    this.workspace?.classList.add("is-active");
    showToast(this.toastHost, "Rig editor ready", "success");
  }

  _logPhase(phase, message, status = "info") {
    const timestamp = formatTimestamp();
    const entry = document.createElement("div");
    entry.className = `rig-editor__log-entry rig-editor__log-entry--${status}`;
    entry.dataset.phase = phase;
    entry.textContent = `[${timestamp}] ${phase.toUpperCase()} — ${message}`;
    this.logList?.appendChild(entry);
    if (this.logList) {
      this.logList.scrollTop = this.logList.scrollHeight;
    }
    const consoleMethod =
      status === "error" ? "error" : status === "success" ? "info" : "log";
    (console[consoleMethod] ?? console.log)(
      `[RigEditor][${timestamp}] ${phase}: ${message}`
    );
  }

  dispose() {
    if (this._disposed) {
      return;
    }
    this._disposed = true;

    if (this._sceneDisposeObserver && this.scene?.onDisposeObservable) {
      this.scene.onDisposeObservable.remove(this._sceneDisposeObserver);
      this._sceneDisposeObserver = null;
    }

    this.gateForm?.removeEventListener("submit", this._gateHandler);
    this.testButton?.removeEventListener("click", this._testButtonHandler);

    while (this._babylonBindings.length > 0) {
      const disposer = this._babylonBindings.pop();
      try {
        disposer?.();
      } catch (error) {
        console.warn("RigEditor binding cleanup failed", error);
      }
    }

    this.root?.remove();
    this.root = null;
    this.toastHost = null;
    this.bootPanel = null;
    this.workspace = null;
    this.timeline = null;
    this.timelineContent = null;
    this.dopeSheet = null;
    this.dopeSheetContent = null;
    this.gateForm = null;
    this.gateInput = null;
    this.gateSubmit = null;
    this.testButton = null;
    this.scene = null;
    this.host = null;
  }
}

/**
 * @param {BABYLON.Scene | null} scene
 * @param {{ host?: HTMLElement | null }} options
 */
export function createRigEditor(scene, options = {}) {
  return new RigEditor(scene, options);
}
