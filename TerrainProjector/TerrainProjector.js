// TerrainProjector script for Cheetah3D
//
// move into ~/Library/Application Support/Cheetah3D/Scripts/Tag
//
// Applies to a mesh (e.g., a flat road map) and projects its
// vertices onto a target terrain mesh along the Y (up) axis.
//
// Usage:
// 1. Select your road/map mesh
// 2. tools -> script -> tag -> TerrainProjector.js
// 3. In inspector, type the exact name of your terrain mesh into "Terrain Name"
// 4. Press the "Project" button in inspector

function buildUI(tool) {
    tool.addParameterSeparator("Terrain Projector");
    tool.addParameterString("Terrain Name", "Terrain");
    tool.addParameterFloat("Y Offset", 0.0, -10.0, 10.0, false, false);
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

// Find the highest world-space Y by scanning all vertices directly.
function findMaxY(meshObj) {
    var core = meshObj.modCore();
    if (!core) core = meshObj.core();
    var matrix = meshObj.obj2WorldMatrix();
    var maxY = -Infinity;
    var vertCount = core.vertexCount();

    for (var i = 0; i < vertCount; i++) {
        var wv = matrix.multiply(core.vertex(i));
        if (wv.y > maxY) maxY = wv.y;
    }

    return maxY;
}

// Collect all world-space triangles from a polygon mesh,
// handling tris, quads and n-gons (fan triangulation).
function collectTriangles(meshObj) {
    var core = meshObj.modCore();
    if (!core) core = meshObj.core();
    var polyCount = core.polygonCount();
    var tris = [];
    var matrix = meshObj.obj2WorldMatrix();

    for (var p = 0; p < polyCount; p++) {
        var polySize = core.polygonSize(p);
        for (var t = 0; t < polySize - 2; t++) {
            var v0 = matrix.multiply(core.vertex(core.vertexIndex(p, 0)));
            var v1 = matrix.multiply(core.vertex(core.vertexIndex(p, t + 1)));
            var v2 = matrix.multiply(core.vertex(core.vertexIndex(p, t + 2)));
            tris.push([v0, v1, v2]);
        }
    }

    return tris;
}

// Möller-Trumbore ray-triangle intersection.
// Ray shoots straight down: origin P, direction D=(0,-1,0).
// All cross products computed explicitly - no algebraic shortcuts.
// Returns t (distance) or -1 if no intersection.
function rayTriangleIntersect(P, v0, v1, v2) {
    var EPSILON   = 1e-8;
    var TOLERANCE = -1e-6;

    var Dx = 0.0;
    var Dy = -1.0;
    var Dz = 0.0;

    var e1x = v1.x - v0.x;
    var e1y = v1.y - v0.y;
    var e1z = v1.z - v0.z;

    var e2x = v2.x - v0.x;
    var e2y = v2.y - v0.y;
    var e2z = v2.z - v0.z;

    // h = D x e2 (full cross product)
    var hx = Dy * e2z - Dz * e2y;
    var hy = Dz * e2x - Dx * e2z;
    var hz = Dx * e2y - Dy * e2x;

    var a = e1x*hx + e1y*hy + e1z*hz;
    if (a > -EPSILON && a < EPSILON) return -1;

    var f = 1.0 / a;

    var sx = P.x - v0.x;
    var sy = P.y - v0.y;
    var sz = P.z - v0.z;

    var u = f * (sx*hx + sy*hy + sz*hz);
    if (u < TOLERANCE || u > 1.0 - TOLERANCE) return -1;

    // q = s x e1 (full cross product)
    var qx = sy*e1z - sz*e1y;
    var qy = sz*e1x - sx*e1z;
    var qz = sx*e1y - sy*e1x;

    // v = f * (D . q) (full dot product)
    var v = f * (Dx*qx + Dy*qy + Dz*qz);
    if (v < TOLERANCE || u + v > 1.0 - TOLERANCE) return -1;

    // t = f * (e2 . q)
    var t = f * (e2x*qx + e2y*qy + e2z*qz);
    if (t < EPSILON) return -1;

    return t;
}

// project a single point downward from maxY onto terrain.
// (returns the Y of the hit, or null if no triangle was hit.)
function projectPoint(worldPos, maxY, triangles) {
    var P = new Vec3D(worldPos.x, maxY + 0.001, worldPos.z);
    var bestY = null;

    for (var i = 0; i < triangles.length; i++) {
        var tri = triangles[i];
        var t = rayTriangleIntersect(P, tri[0], tri[1], tri[2]);
        if (t >= 0) {
            var hitY = P.y - t;
            if (bestY === null || hitY > bestY) {
                bestY = hitY;
            }
        }
    }

    return bestY;
}

// Main button callback
function projectOntoTerrain(tool) {
    var doc         = tool.document();
    var roadObj     = doc.selectedObject();
    var terrainName = tool.getParameter("Terrain Name");
    var yOffset     = tool.getParameter("Y Offset");

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
        OS.messageBox("Error", "Could not find object named '" + terrainName + "' in scene.");
        return;
    }

    var maxY = findMaxY(terrainObj);
    print("TerrainProjector: Terrain max Y = " + maxY + " (ray starts at " + (maxY + 0.001) + ")");

    var triangles = collectTriangles(terrainObj);
    if (triangles.length === 0) {
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

    var hits   = 0;
    var misses = 0;

    for (var i = 0; i < vertCount; i++) {
        var localPos = roadCore.vertex(i);
        var worldPos = roadMatrix.multiply(localPos);
        var hitY     = projectPoint(worldPos, maxY, triangles);

        if (hitY !== null) {
            var newWorldPos = new Vec3D(worldPos.x, hitY + yOffset, worldPos.z);
            var newLocalPos = roadMatrixInv.multiply(newWorldPos);
            roadCore.setVertex(i, newLocalPos);
            hits++;
        } else {
            misses++;
        }
    }

    roadObj.update();
    doc.redrawAll();

    print("TerrainProjector: " + hits + " vertices projected, " + misses + " missed (no terrain hit).");
}

function run(tool) {
}
