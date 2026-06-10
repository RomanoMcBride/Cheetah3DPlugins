// TerrainProjector script for Cheetah3D
//
// Applies to a mesh (e.g., a flat road map) and projects its
// vertices onto a target terrain mesh along the Y (up) axis.
//
// Usage:
// 1. Select your road/map mesh
// 2. tools -> script -> tag -> TerrainProjector.js
// 3. In inspector, drag your terrain mesh into the "Terrain" link slot
// 4. Press the "Project" button in inspector
//
// Assumes the terrain is a regular grid mesh aligned to XZ.
// Uses bilinear interpolation on a flat heightmap for O(1) height
// lookup per vertex. Points outside terrain bounds are clamped to
// the nearest terrain edge.

function buildUI(tool) {
    tool.addParameterSeparator("Terrain Projector");
    tool.addParameterLink("Terrain", false);
    tool.addParameterFloat("Y Offset", 0.0, -10.0, 10.0, false, false);
    tool.addParameterSeparator("Building Mode");
    tool.addParameterBool("Group Connected", false, false, false, false, false);
    tool.addParameterButton("Project", "Project onto Terrain", "projectOntoTerrain");
}

// Try to fill a heightmap with given cols/rows, return collision count.
// If collisions is 0 the orientation is correct.
function tryFillHeightmap(wx, wy, wz, vertCount, cols, rows, minX, minZ, stepX, stepZ) {
    var heightmap = [];
    var filled    = [];
    for (var i = 0; i < cols * rows; i++) {
        heightmap.push(0.0);
        filled.push(false);
    }

    var collisions = 0;
    var maxY       = -Infinity;

    for (var i = 0; i < vertCount; i++) {
        var xi  = Math.round((wx[i] - minX) / stepX);
        var zi  = Math.round((wz[i] - minZ) / stepZ);
        // Clamp to valid range
        if (xi < 0) xi = 0; if (xi >= cols) xi = cols - 1;
        if (zi < 0) zi = 0; if (zi >= rows) zi = rows - 1;
        var idx = zi * cols + xi;
        if (filled[idx]) collisions++;
        filled[idx]    = true;
        heightmap[idx] = wy[i];
        if (wy[i] > maxY) maxY = wy[i];
    }

    return { heightmap: heightmap, collisions: collisions, maxY: maxY };
}

// Build a flat heightmap from a regular XZ-aligned terrain mesh.
// Tries both possible grid orientations (cols x rows and rows x cols)
// and picks the one with fewest collisions.
// Grid vertex (xi, zi) -> Y stored at heightmap[zi * cols + xi].
function buildHeightmap(meshObj) {
    var core = meshObj.modCore();
    if (!core) core = meshObj.core();
    var matrix    = meshObj.obj2WorldMatrix();
    var vertCount = core.vertexCount();
    var polyCount = core.polygonCount();

    var minX =  Infinity, maxX = -Infinity;
    var minZ =  Infinity, maxZ = -Infinity;

    var wx = [], wy = [], wz = [];
    for (var i = 0; i < vertCount; i++) {
        var wv = matrix.multiply(core.vertex(i));
        wx.push(wv.x);
        wy.push(wv.y);
        wz.push(wv.z);
        if (wv.x < minX) minX = wv.x;
        if (wv.x > maxX) maxX = wv.x;
        if (wv.z < minZ) minZ = wv.z;
        if (wv.z > maxZ) maxZ = wv.z;
    }

    // Derive cols candidates from quadratic formula.
    // vertCount = cols * rows, polyCount = (cols-1)*(rows-1)
    // => cols^2 - (V - P + 1)*cols + V = 0
    var b            = -(vertCount - polyCount + 1);
    var discriminant = b*b - 4*vertCount;
    var sq           = Math.sqrt(Math.max(0, discriminant));
    var c1           = Math.round((-b + sq) / 2);
    var c2           = Math.round((-b - sq) / 2);

    // Also try the row-counting method as a third candidate
    var MERGE_TOL = 1e-5;
    var colsByRow = 0;
    for (var i = 0; i < vertCount; i++) {
        if (Math.abs(wz[i] - minZ) < MERGE_TOL) colsByRow++;
    }

    // Collect unique candidates (cols and their transpose rows)
    var candidates = [];
    function addCandidate(c) {
        if (c > 1 && vertCount % c === 0) candidates.push(c);
    }
    addCandidate(c1);
    addCandidate(c2);
    addCandidate(Math.round(vertCount / c1)); // transpose of c1
    addCandidate(Math.round(vertCount / c2)); // transpose of c2
    addCandidate(colsByRow);
    addCandidate(Math.round(vertCount / colsByRow));

    print("TerrainProjector: Trying " + candidates.length + " grid orientations...");

    // Try each candidate and pick the one with fewest collisions
    var bestResult = null;
    var bestCols   = 0;
    var bestRows   = 0;
    var bestStepX  = 0;
    var bestStepZ  = 0;

    for (var ci = 0; ci < candidates.length; ci++) {
        var cols  = candidates[ci];
        var rows  = Math.round(vertCount / cols);
        var stepX = (maxX - minX) / (cols - 1);
        var stepZ = (maxZ - minZ) / (rows - 1);

        if (stepX <= 0 || stepZ <= 0) continue;

        var result = tryFillHeightmap(wx, wy, wz, vertCount, cols, rows,
                                      minX, minZ, stepX, stepZ);

        print("  cols=" + cols + " rows=" + rows
            + " stepX=" + stepX.toFixed(4) + " stepZ=" + stepZ.toFixed(4)
            + " -> collisions=" + result.collisions);

        if (bestResult === null || result.collisions < bestResult.collisions) {
            bestResult = result;
            bestCols   = cols;
            bestRows   = rows;
            bestStepX  = stepX;
            bestStepZ  = stepZ;
        }

        // Perfect solution found - no need to try more
        if (result.collisions === 0) break;
    }

    print("TerrainProjector: Best orientation: " + bestCols + "x" + bestRows
        + " verts (" + (bestCols-1) + "x" + (bestRows-1) + " cells)"
        + ", stepX=" + bestStepX.toFixed(5)
        + ", stepZ=" + bestStepZ.toFixed(5)
        + ", collisions=" + bestResult.collisions);

    return {
        heightmap: bestResult.heightmap,
        minX: minX, minZ: minZ,
        stepX: bestStepX, stepZ: bestStepZ,
        cols: bestCols, rows: bestRows,
        maxY: bestResult.maxY
    };
}

// Sample terrain height at world (x, z) using bilinear interpolation.
// The quad is split along the BL-TR diagonal to match the mesh topology.
// Points outside terrain bounds are clamped to the nearest terrain edge,
// effectively extending the heightmap infinitely in all directions.
function sampleHeight(hm, x, z) {
    var gx = (x - hm.minX) / hm.stepX;
    var gz = (z - hm.minZ) / hm.stepZ;

    // Clamp to valid cell range
    gx = Math.max(0, Math.min(gx, hm.cols - 1 - 1e-10));
    gz = Math.max(0, Math.min(gz, hm.rows - 1 - 1e-10));

    var cx = Math.floor(gx);
    var cz = Math.floor(gz);

    var fx = gx - cx;
    var fz = gz - cz;

    var h00 = hm.heightmap[ cz      * hm.cols + cx    ]; // BL
    var h10 = hm.heightmap[ cz      * hm.cols + cx + 1]; // BR
    var h01 = hm.heightmap[(cz + 1) * hm.cols + cx    ]; // TL
    var h11 = hm.heightmap[(cz + 1) * hm.cols + cx + 1]; // TR

    // Lower-right triangle (BL, BR, TR): fz < 1 - fx
    // Upper-left triangle  (BL, TR, TL): fz >= 1 - fx
    if (fz < 1.0 - fx) {
        return h00 + fx * (h10 - h00) + fz * (h11 - h10);
    } else {
        return h01 + fx * (h11 - h01) + (1.0 - fz) * (h00 - h01);
    }
}

// Build a vertex adjacency list using a hash set for deduplication.
function buildVertexAdjacency(core) {
    var vertCount = core.vertexCount();
    var polyCount = core.polygonCount();
    var edgeSet   = {};
    var adj       = [];
    for (var i = 0; i < vertCount; i++) adj.push([]);

    for (var p = 0; p < polyCount; p++) {
        var polySize  = core.polygonSize(p);
        var polyVerts = [];
        for (var c = 0; c < polySize; c++) polyVerts.push(core.vertexIndex(p, c));
        for (var a = 0; a < polyVerts.length; a++) {
            for (var b = 0; b < polyVerts.length; b++) {
                if (a === b) continue;
                var va  = polyVerts[a], vb = polyVerts[b];
                var key = va + "_" + vb;
                if (!edgeSet[key]) {
                    edgeSet[key] = true;
                    adj[va].push(vb);
                }
            }
        }
    }
    return adj;
}

// Find connected components via iterative BFS flood fill.
// Returns an array of components, each being an array of vertex indices.
function findConnectedComponents(core) {
    var vertCount = core.vertexCount();
    var adj       = buildVertexAdjacency(core);
    var visited   = [];
    for (var i = 0; i < vertCount; i++) visited.push(false);

    var components = [];
    for (var start = 0; start < vertCount; start++) {
        if (visited[start]) continue;
        var component  = [];
        var queue      = [start];
        visited[start] = true;
        while (queue.length > 0) {
            var current    = queue.shift();
            component.push(current);
            var neighbours = adj[current];
            for (var n = 0; n < neighbours.length; n++) {
                var nb = neighbours[n];
                if (!visited[nb]) { visited[nb] = true; queue.push(nb); }
            }
        }
        components.push(component);
    }
    return components;
}

function projectOntoTerrain(tool) {
    var doc            = tool.document();
    var roadObj        = doc.selectedObject();
    var terrainObj     = tool.getParameter("Terrain");
    var yOffset        = tool.getParameter("Y Offset");
    var groupConnected = tool.getParameter("Group Connected");

    if (!roadObj) {
        OS.messageBox("Error", "Please select the road/map mesh first.");
        return;
    }
    if (!terrainObj) {
        OS.messageBox("Error", "Please drag a terrain mesh into the Terrain slot.");
        return;
    }

    print("TerrainProjector: Building heightmap...");
    var hm = buildHeightmap(terrainObj);
    print("TerrainProjector: Terrain max Y = " + hm.maxY);

    var roadCore = roadObj.core();
    if (!roadCore || roadCore.vertexCount() === 0) {
        OS.messageBox("Error", "Road mesh has no vertices.");
        return;
    }

    var roadMatrix    = roadObj.obj2WorldMatrix();
    var roadMatrixInv = roadMatrix.inverse();
    var vertCount     = roadCore.vertexCount();

    roadObj.recordGeometryForUndo();

    var hits = 0, misses = 0;

    if (!groupConnected) {
        for (var i = 0; i < vertCount; i++) {
            var worldPos = roadMatrix.multiply(roadCore.vertex(i));
            var hitY     = sampleHeight(hm, worldPos.x, worldPos.z);
            if (hitY !== null) {
                var nwp = new Vec3D(worldPos.x, hitY + yOffset, worldPos.z);
                roadCore.setVertex(i, roadMatrixInv.multiply(nwp));
                hits++;
            } else {
                misses++;
            }
        }
    } else {
        print("TerrainProjector: Finding connected components...");
        var components = findConnectedComponents(roadCore);
        print("TerrainProjector: Found " + components.length + " components.");

        for (var c = 0; c < components.length; c++) {
            var component = components[c];
            var sumX = 0.0, sumZ = 0.0, minY = Infinity;

            for (var vi = 0; vi < component.length; vi++) {
                var wp = roadMatrix.multiply(roadCore.vertex(component[vi]));
                sumX += wp.x;
                sumZ += wp.z;
                if (wp.y < minY) minY = wp.y;
            }

            var hitY = sampleHeight(
                hm,
                sumX / component.length,
                sumZ / component.length
            );

            if (hitY !== null) {
                var deltaY = (hitY + yOffset) - minY;
                for (var vi = 0; vi < component.length; vi++) {
                    var wp  = roadMatrix.multiply(roadCore.vertex(component[vi]));
                    var nwp = new Vec3D(wp.x, wp.y + deltaY, wp.z);
                    roadCore.setVertex(component[vi], roadMatrixInv.multiply(nwp));
                }
                hits += component.length;
            } else {
                misses += component.length;
            }
        }
    }

    roadObj.update();
    doc.redrawAll();
    print("TerrainProjector: " + hits + " vertices projected, "
        + misses + " missed (no terrain hit).");
}

function run(tool) {
}
