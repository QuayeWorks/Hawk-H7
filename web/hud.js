/**
 * Simple QA panel that records scene resource counts before and after
 * transitions. The panel renders to the provided host node.
 */
export class SceneAuditPanel {
  /**
   * @param {HTMLElement} host
   */
  constructor(host) {
    if (!host) {
      throw new Error("SceneAuditPanel host element is required");
    }

    this._host = host;
    this._tableBody = document.createElement("tbody");
    this._rows = [];

    const heading = document.createElement("h2");
    heading.textContent = "Scene Audit";

    const description = document.createElement("p");
    description.textContent =
      "Tracks Babylon.js resources before and after each mode switch to catch leaks.";
    description.style.marginTop = "0";
    description.style.marginBottom = "12px";
    description.style.fontSize = "0.75rem";
    description.style.opacity = "0.75";

    const table = document.createElement("table");
    table.className = "audit-table";

    const header = document.createElement("thead");
    header.innerHTML = `
      <tr>
        <th>From → To</th>
        <th>Nodes</th>
        <th>Materials</th>
        <th>Textures</th>
      </tr>
    `;

    table.append(header, this._tableBody);
    host.append(heading, description, table);
  }

  /**
   * Records a transition in the panel.
   *
   * @param {string | null} fromMode
   * @param {string} toMode
   * @param {{ nodes: number, materials: number, textures: number }} preCounts
   * @param {{ nodes: number, materials: number, textures: number }} postCounts
   */
  recordTransition(fromMode, toMode, preCounts, postCounts) {
    const transition = `${fromMode ?? "—"} → ${toMode}`;
    const row = document.createElement("tr");
    row.innerHTML = `
      <td class="mode">${transition}</td>
      <td>${preCounts.nodes} → ${postCounts.nodes}</td>
      <td>${preCounts.materials} → ${postCounts.materials}</td>
      <td>${preCounts.textures} → ${postCounts.textures}</td>
    `;

    this._rows.push(row);
    this._tableBody.appendChild(row);

    if (this._rows.length > 10) {
      const removed = this._rows.shift();
      removed?.remove();
    }
  }
}

/**
 * Convenience helper for creating the panel with minimal ceremony.
 *
 * @param {HTMLElement} host
 * @returns {SceneAuditPanel}
 */
export function createHUD(host) {
  return new SceneAuditPanel(host);
}
