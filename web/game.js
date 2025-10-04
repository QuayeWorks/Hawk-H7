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
