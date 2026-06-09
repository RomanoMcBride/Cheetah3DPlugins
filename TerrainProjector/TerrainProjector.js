// TerrainProjector script for Cheetah3D
//
// Applies to a mesh (e.g., a flat road map) and projects its
// vertices onto a target terrain mesh along the Y (up) axis.
//
// Usage:
// 1. Select your road/map mesh
// 2. tools -> script -> tag -> TerrainProjector.js
// 3. In inspector, type the exact name of your terrain mesh into "Terrain Name"
// 4. Press the "Project" button in inspector
// 
//
// Assumes the terrain is a regular grid mesh aligned to XZ.
// Uses direct cell lookup for O(1) triangle retrieval per vertex.

function buildUI(tool) {
    tool.addParameterSeparator("Terrain Projector");
    tool.addParameterString("Terrain Name", "Terrain");
    tool.addParameterFloat("Y Offset", 0.0, -10.0, 10.0, false, false);
    tool.addParameterSeparator("Building Mode");
    tool.addParameterBool("Group Connected", false, false, false, false, false);
    tool.addParameterButton("Project", "Project onto Terrain", "projectOntoTerrain");
}

// Recursively search the scene graph for an object by name.
function findObjectByName(obj, name) {
    if (obj.getParameter("name") === name) return obj;
    for (var i = 0; i < obj.childCount(); i++) {
        var found = findObjectByName(obj.childAtIndex(i), name);
        if (found) return found;
    }
    return null;
}

// Find the highest world-space Y by scanning ALL vertices.
function findMaxY(meshObj) {
    var core = meshObj.modCore();
    if (!core) core = meshObj.core();
    var matrix    = meshObj.obj2WorldMatrix();
    var maxY      = -Infinity;
    var vertCount = core.vertexCount();
    for (var i = 0; i < vertCount; i++) {
        var wv = matrix.multiply(core.vertex(i));
        if (wv.y > maxY) maxY = wv.y;
    }
    return maxY;
}

// Build a regular grid index from a terrain mesh.
//
// Figures out the grid resolution (cols x rows) by finding the
// unique sorted X and Z vertex positions. Works for any regular
// grid regardless of spacing or world transform.
//
// Returns:
// {
//   tris,       // array of [v0,v1,v2] (world-space Vec3D)
//               // laid out as: cell(cx,cz) = tris at indices
//               //   (cz * cols + cx) * 2 + 0  (lower-left tri)
//               //   (cz * cols + cx) * 2 + 1  (upper-right tri)
//   xs,         // sorted unique X values of grid vertices
//   zs,         // sorted unique Z values of grid vertices
//   cols,       // number of cells in X  = xs.length - 1
//   rows        // number of cells in Z  = zs.length - 1
// }
function buildRegularGridIndex(meshObj) {
    var core = meshObj.modCore();
    if (!core) core = meshObj.core();
    var matrix    = meshObj.obj2WorldMatrix();
    var vertCount = core.vertexCount();

    // --- Collect all world-space vertex positions ---
    var verts = [];
    for (var i = 0; i < vertCount; i++) {
        var wv = matrix.multiply(core.vertex(i));
        verts.push(wv);
    }

    // --- Find unique sorted X and Z values ---
    // Use a tolerance to merge near-identical floats
    var MERGE_TOL = 1e-5;

    function collectUnique(vals) {
        vals.sort(function(a, b) { return a - b; });
        var unique = [vals[0]];
        for (var i = 1; i < vals.length; i++) {
            if (vals[i] - unique[unique.length - 1] > MERGE_TOL) {
                unique.push(vals[i]);
            }
        }
        return unique;
    }

    var allX = [], allZ = [];
    for (var i = 0; i < verts.length; i++) {
        allX.push(verts[i].x);
        allZ.push(verts[i].z);
    }
    var xs = collectUnique(allX);
    var zs = collectUnique(allZ);

    var cols = xs.length - 1; // cells in X
    var rows = zs.length - 1; // cells in Z

    print("TerrainProjector: Grid " + xs.length + "x" + zs.length
        + " vertices = " + cols + "x" + rows + " cells ("
        + (cols * rows * 2) + " triangles)");

    // --- Build a vertex lookup: (xi, zi) -> world-space Vec3D ---
    // For each vertex, find its xi and zi indices in xs/zs.
    // Binary search for speed.
    function bisect(arr, val) {
        var lo = 0, hi = arr.length - 1;
        while (lo < hi) {
            var mid = (lo + hi) >> 1;
            if (arr[mid] < val - MERGE_TOL) lo = mid + 1;
            else hi = mid;
        }
        return lo;
    }

    // vertGrid[zi][xi] = world-space Vec3D
    var vertGrid = [];
    for (var zi = 0; zi < zs.length; zi++) {
        var row = [];
        for (var xi = 0; xi < xs.length; xi++) row.push(null);
        vertGrid.push(row);
    }

    for (var i = 0; i < verts.length; i++) {
        var xi = bisect(xs, verts[i].x);
        var zi = bisect(zs, verts[i].z);
        vertGrid[zi][xi] = verts[i];
    }

    // --- Build triangle array: 2 tris per cell, ordered by cell ---
    // Each quad cell (cx, cz) has corners:
    //   BL = vertGrid[cz  ][cx  ]
    //   BR = vertGrid[cz  ][cx+1]
    //   TL = vertGrid[cz+1][cx  ]
    //   TR = vertGrid[cz+1][cx+1]
    //
    // Split into:
    //   tri0: BL, BR, TR  (lower-right triangle)
    //   tri1: BL, TR, TL  (upper-left triangle)
    //
    // Cell (cx,cz) -> tris[(cz*cols+cx)*2] and [*2+1]

    var tris = [];
    for (var cz = 0; cz < rows; cz++) {
        for (var cx = 0; cx < cols; cx++) {
            var BL = vertGrid[cz  ][cx  ];
            var BR = vertGrid[cz  ][cx+1];
            var TL = vertGrid[cz+1][cx  ];
            var TR = vertGrid[cz+1][cx+1];
            tris.push([BL, BR, TR]); // tri0
            tris.push([BL, TR, TL]); // tri1
        }
    }

    return { tris: tris, xs: xs, zs: zs, cols: cols, rows: rows };
}

// For a point (x, z), return triangle indices to test.
// Tests the cell it falls in plus its 8 neighbours (3x3 = 18 tris)
// to handle floating point edge cases.
function getCandidateTriIndices(index, x, z) {
    // Binary search for cell
    function bisect(arr, val) {
        var lo = 0, hi = arr.length - 2; // hi = last cell index
        while (lo < hi) {
            var mid = (lo + hi + 1) >> 1;
            if (arr[mid] <= val) lo = mid;
            else hi = mid - 1;
        }
        return lo;
    }

    var cx = bisect(index.xs, x);
    var cz = bisect(index.zs, z);

    var result = [];
    // 3x3 neighbourhood
    for (var dz = -1; dz <= 1; dz++) {
        for (var dx = -1; dx <= 1; dx++) {
            var nx = cx + dx;
            var nz = cz + dz;
            if (nx >= 0 && nx < index.cols && nz >= 0 && nz < index.rows) {
                var base = (nz * index.cols + nx) * 2;
                result.push(base);     // tri0
                result.push(base + 1); // tri1
            }
        }
    }
    return result;
}

// Möller-Trumbore ray-triangle intersection.
// Ray shoots straight down: origin P, direction D=(0,-1,0).
// Returns t (distance) or -1 if no intersection.
function rayTriangleIntersect(P, v0, v1, v2) {
    var EPSILON   = 1e-8;
    var TOLERANCE = -1e-6;

    var Dx = 0.0, Dy = -1.0, Dz = 0.0;

    var e1x = v1.x - v0.x, e1y = v1.y - v0.y, e1z = v1.z - v0.z;
    var e2x = v2.x - v0.x, e2y = v2.y - v0.y, e2z = v2.z - v0.z;

    var hx = Dy*e2z - Dz*e2y;
    var hy = Dz*e2x - Dx*e2z;
    var hz = Dx*e2y - Dy*e2x;

    var a = e1x*hx + e1y*hy + e1z*hz;
    if (a > -EPSILON && a < EPSILON) return -1;

    var f  = 1.0 / a;
    var sx = P.x - v0.x, sy = P.y - v0.y, sz = P.z - v0.z;

    var u = f * (sx*hx + sy*hy + sz*hz);
    if (u < TOLERANCE || u > 1.0 - TOLERANCE) return -1;

    var qx = sy*e1z - sz*e1y;
    var qy = sz*e1x - sx*e1z;
    var qz = sx*e1y - sy*e1x;

    var v = f * (Dx*qx + Dy*qy + Dz*qz);
    if (v < TOLERANCE || u + v > 1.0 - TOLERANCE) return -1;

    var t = f * (e2x*qx + e2y*qy + e2z*qz);
    if (t < EPSILON) return -1;

    return t;
}

// ---------------------------------------------------------------
// Project a single XZ point downward using the grid index.
// Returns the Y of the hit, or null if no triangle was hit.
// ---------------------------------------------------------------
function projectPoint(x, z, maxY, index) {
    var P          = new Vec3D(x, maxY + 0.001, z);
    var candidates = getCandidateTriIndices(index, x, z);
    var bestY      = null;

    for (var i = 0; i < candidates.length; i++) {
        var tri = index.tris[candidates[i]];
        var t   = rayTriangleIntersect(P, tri[0], tri[1], tri[2]);
        if (t >= 0) {
            var hitY = P.y - t;
            if (bestY === null || hitY > bestY) bestY = hitY;
        }
    }

    return bestY;
}

// Build vertex adjacency list from polygon data.
function buildVertexAdjacency(core) {
    var vertCount = core.vertexCount();
    var polyCount = core.polygonCount();
    var adj = [];
    for (var i = 0; i < vertCount; i++) adj.push([]);

    for (var p = 0; p < polyCount; p++) {
        var polySize  = core.polygonSize(p);
        var polyVerts = [];
        for (var c = 0; c < polySize; c++) polyVerts.push(core.vertexIndex(p, c));
        for (var a = 0; a < polyVerts.length; a++) {
            for (var b = 0; b < polyVerts.length; b++) {
                if (a === b) continue;
                var va = polyVerts[a], vb = polyVerts[b];
                var found = false;
                for (var k = 0; k < adj[va].length; k++) {
                    if (adj[va][k] === vb) { found = true; break; }
                }
                if (!found) adj[va].push(vb);
            }
        }
    }
    return adj;
}

// Find connected components via BFS.
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

// Main button callback
function projectOntoTerrain(tool) {
    var doc            = tool.document();
    var roadObj        = doc.selectedObject();
    var terrainName    = tool.getParameter("Terrain Name");
    var yOffset        = tool.getParameter("Y Offset");
    var groupConnected = tool.getParameter("Group Connected");

    if (!roadObj) {
        OS.messageBox("Error", "Please select the road/map mesh first.");
        return;
    }
    if (!terrainName || terrainName === "") {
        OS.messageBox("Error", "Please enter the terrain mesh name.");
        return;
    }

    var terrainObj = findObjectByName(doc.root(), terrainName);
    if (!terrainObj) {
        OS.messageBox("Error", "Could not find object named '" + terrainName + "'.");
        return;
    }

    var maxY = findMaxY(terrainObj);
    print("TerrainProjector: Terrain max Y = " + maxY);

    var index = buildRegularGridIndex(terrainObj);
    if (index.tris.length === 0) {
        OS.messageBox("Error", "Terrain mesh has no polygons.");
        return;
    }

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
        // Simple mode: project each vertex independently
        for (var i = 0; i < vertCount; i++) {
            var worldPos = roadMatrix.multiply(roadCore.vertex(i));
            var hitY     = projectPoint(worldPos.x, worldPos.z, maxY, index);
            if (hitY !== null) {
                var nwp = new Vec3D(worldPos.x, hitY + yOffset, worldPos.z);
                roadCore.setVertex(i, roadMatrixInv.multiply(nwp));
                hits++;
            } else {
                misses++;
            }
        }
    } else {
        // Building mode: one ray per connected component
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

            var hitY = projectPoint(
                sumX / component.length,
                sumZ / component.length,
                maxY, index
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
