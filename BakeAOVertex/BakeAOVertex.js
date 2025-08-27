// Vertex ambient occlusion baker script for Cheetah3D
// Intersection based on Möller-Trumbore, 1997
// Spherical monte carlo integration
// Bakes ambient occlusion into UV (because we can't modify via javascript yet via Cheetah3D API)

function buildUI(tool) {
	tool.addParameterSeparator("Ambient Occlusion Baker");
	tool.addParameterInt("Sample Rays", 32, 8, 128, false, false);
	tool.addParameterFloat("Max Distance", 1.0, 0.1, 10.0, false, false);
	tool.addParameterFloat("Bias", 0.01, 0.001, 0.1, false, false);
	tool.addParameterFloat("Intensity", 1.0, 0.1, 2.0, false, false);
	tool.addParameterBool("Invert", false, false, false, false, false);
	tool.addParameterButton("Bake AO", "Bake", "bakeAmbientOcclusion");
}

function bakeAmbientOcclusion(tool) {
	var doc = tool.document();
	var selectedObj = doc.selectedObject();
	
	if (!selectedObj) {
		OS.messageBox("Error", "Please select an object first.");
		return;
	}
	
	var sampleRays = tool.getParameter("Sample Rays");
	var maxDistance = tool.getParameter("Max Distance");
	var bias = tool.getParameter("Bias");
	var intensity = tool.getParameter("Intensity");
	var invert = tool.getParameter("Invert");
	
	print("Starting Ambient Occlusion baking...");
	print("Sample Rays: " + sampleRays);
	print("Max Distance: " + maxDistance);
	
	// Collect all meshes in the scene for ray intersection
	var allMeshes = [];
	collectAllMeshes(doc.root(), allMeshes);
	
	// Process all meshes under selected object
	var processedCount = 0;
	processedCount = processMeshesRecursively(selectedObj, allMeshes, sampleRays, maxDistance, bias, intensity, invert, processedCount);
	
	doc.redrawAll();
	print("Ambient Occlusion baking complete. Processed " + processedCount + " meshes.");
}

function collectAllMeshes(obj, meshArray) {
	if (obj.family() == NGONFAMILY) {
		var core = obj.modCore();
		if (core && core.vertexCount() > 0) {
			meshArray.push({
				object: obj,
				core: core,
				matrix: obj.obj2WorldMatrix()
			});
		}
	}
	
	// Recursively collect from children
	for (var i = 0; i < obj.childCount(); i++) {
		collectAllMeshes(obj.childAtIndex(i), meshArray);
	}
}

function processMeshesRecursively(obj, allMeshes, sampleRays, maxDistance, bias, intensity, invert, count) {
	if (obj.family() == NGONFAMILY) {
		var core = obj.core();
		if (core && core.vertexCount() > 0) {
			obj.recordGeometryForUndo();
			bakeMeshAO(obj, core, allMeshes, sampleRays, maxDistance, bias, intensity, invert);
			obj.update();
			count++;
			print("Processed mesh: " + obj.getParameter("name"));
		}
	}
	
	// Process children
	for (var i = 0; i < obj.childCount(); i++) {
		count = processMeshesRecursively(obj.childAtIndex(i), allMeshes, sampleRays, maxDistance, bias, intensity, invert, count);
	}
	
	return count;
}

function bakeMeshAO(meshObj, core, allMeshes, sampleRays, maxDistance, bias, intensity, invert) {
	var meshMatrix = meshObj.obj2WorldMatrix();
	var vertexCount = core.vertexCount();
	
	// Generate hemisphere sample directions
	var sampleDirections = generateHemisphereSamples(sampleRays);
	
	for (var v = 0; v < vertexCount; v++) {
		var vertex = core.vertex(v);
		var worldVertex = meshMatrix.multiply(vertex);
		
		// Calculate vertex normal (approximate using nearby faces)
		var normal = calculateVertexNormal(core, v, meshMatrix);
		
		// Cast rays and calculate occlusion
		var occlusion = calculateOcclusion(worldVertex, normal, sampleDirections, allMeshes, maxDistance, bias);
		
		// Apply intensity and invert if needed
		occlusion = Math.pow(occlusion, intensity);
		if (invert) {
			occlusion = 1.0 - occlusion;
		}
		
		// Set vertex color (grayscale AO)
		var aoColor = new Vec4D(occlusion, occlusion, occlusion, 1.0);
		setVertexColor(core, v, aoColor);
	}
}

function generateHemisphereSamples(count) {
	var samples = [];
	
	for (var i = 0; i < count; i++) {
		// Generate random points on hemisphere using spherical coordinates
		var u = Math.random();
		var v = Math.random();
		
		var theta = 2.0 * Math.PI * u; // azimuth
		var phi = Math.acos(Math.sqrt(v)); // elevation (hemisphere)
		
		var x = Math.sin(phi) * Math.cos(theta);
		var y = Math.cos(phi); // up direction
		var z = Math.sin(phi) * Math.sin(theta);
		
		samples.push(new Vec3D(x, y, z));
	}
	
	return samples;
}

function calculateVertexNormal(core, vertexIndex, matrix) {
	var normal = new Vec3D(0, 0, 0);
	var faceCount = 0;
	
	// Find all faces that use this vertex
	for (var p = 0; p < core.polygonCount(); p++) {
		var polySize = core.polygonSize(p);
		for (var c = 0; c < polySize; c++) {
			if (core.vertexIndex(p, c) == vertexIndex) {
				// This face uses our vertex, add its normal
				var faceNormal = core.normal(p);
				normal = normal.add(faceNormal);
				faceCount++;
				break;
			}
		}
	}
	
	if (faceCount > 0) {
		normal = normal.multiply(1.0 / faceCount);
		// Transform to world space (assuming uniform scaling)
		normal = matrix.multiply(normal).sub(matrix.multiply(new Vec3D(0, 0, 0)));
		var length = normal.norm();
		if (length > 0) {
			normal = normal.multiply(1.0 / length);
		}
	} else {
		normal = new Vec3D(0, 1, 0); // default up
	}
	
	return normal;
}

function calculateOcclusion(worldPos, normal, sampleDirections, allMeshes, maxDistance, bias) {
	var occluded = 0;
	var totalSamples = sampleDirections.length;
	
	// Create tangent space basis
	var up = new Vec3D(0, 1, 0);
	if (Math.abs(normal.dot(up)) > 0.9) {
		up = new Vec3D(1, 0, 0);
	}
	
	var tangent = normal.cross(up);
	var length = tangent.norm();
	if (length > 0) {
		tangent = tangent.multiply(1.0 / length);
	}
	var bitangent = normal.cross(tangent);
	
	for (var s = 0; s < totalSamples; s++) {
		var sampleDir = sampleDirections[s];
		
		// Transform sample direction to world space using normal as up vector
		var worldDir = new Vec3D(
			tangent.x * sampleDir.x + normal.x * sampleDir.y + bitangent.x * sampleDir.z,
			tangent.y * sampleDir.x + normal.y * sampleDir.y + bitangent.y * sampleDir.z,
			tangent.z * sampleDir.x + normal.z * sampleDir.y + bitangent.z * sampleDir.z
		);
		
		// Offset ray start by bias along normal
		var rayStart = worldPos.add(normal.multiply(bias));
		var rayEnd = rayStart.add(worldDir.multiply(maxDistance));
		
		// Test intersection with all meshes
		if (rayIntersectsAnyMesh(rayStart, rayEnd, allMeshes)) {
			occluded++;
		}
	}
	
	// Return ambient accessibility (1 - occlusion)
	return 1.0 - (occluded / totalSamples);
}

function rayIntersectsAnyMesh(rayStart, rayEnd, meshes) {
	var rayDir = rayEnd.sub(rayStart);
	var rayLength = rayDir.norm();
	if (rayLength == 0) return false;
	
	rayDir = rayDir.multiply(1.0 / rayLength);
	
	for (var m = 0; m < meshes.length; m++) {
		var mesh = meshes[m];
		var core = mesh.core;
		var invMatrix = mesh.matrix.inverse();
		
		// Transform ray to local space
		var localStart = invMatrix.multiply(rayStart);
		var localEnd = invMatrix.multiply(rayEnd);
		var localDir = localEnd.sub(localStart);
		var localLength = localDir.norm();
		if (localLength == 0) continue;
		localDir = localDir.multiply(1.0 / localLength);
		
		// Test against triangles
		for (var p = 0; p < core.polygonCount(); p++) {
			var polySize = core.polygonSize(p);
			
			// Triangulate polygon and test each triangle
			for (var t = 0; t < polySize - 2; t++) {
				var v0 = core.vertex(core.vertexIndex(p, 0));
				var v1 = core.vertex(core.vertexIndex(p, t + 1));
				var v2 = core.vertex(core.vertexIndex(p, t + 2));
				
				if (rayTriangleIntersect(localStart, localDir, localLength, v0, v1, v2)) {
					return true;
				}
			}
		}
	}
	
	return false;
}

function rayTriangleIntersect(rayStart, rayDir, rayLength, v0, v1, v2) {
	var epsilon = 0.000001;
	
	var edge1 = v1.sub(v0);
	var edge2 = v2.sub(v0);
	var h = rayDir.cross(edge2);
	var a = edge1.dot(h);
	
	if (a > -epsilon && a < epsilon) {
		return false; // Ray is parallel to triangle
	}
	
	var f = 1.0 / a;
	var s = rayStart.sub(v0);
	var u = f * s.dot(h);
	
	if (u < 0.0 || u > 1.0) {
		return false;
	}
	
	var q = s.cross(edge1);
	var v = f * rayDir.dot(q);
	
	if (v < 0.0 || u + v > 1.0) {
		return false;
	}
	
	var t = f * edge2.dot(q);
	
	if (t > epsilon && t < rayLength) {
		return true; // Ray intersects triangle
	}
	
	return false;
}

function setVertexColor(core, vertexIndex, color) {
	// Find all polygon corners that use this vertex and set their colors
	for (var p = 0; p < core.polygonCount(); p++) {
		var polySize = core.polygonSize(p);
		for (var c = 0; c < polySize; c++) {
			if (core.vertexIndex(p, c) == vertexIndex) {
				// Set UV coordinate x only to encode color (this is a workaround)
				// In newer versions of Cheetah3D, we might have direct vertex color support? Please? :)
				core.setUVCoord(p, c, new Vec4D(color.x, 0, 0, 0));
			}
		}
	}
}
