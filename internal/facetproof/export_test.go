package facetproof

// AuditFacetedMeshWorkers runs the audit with an explicit worker count.
var AuditFacetedMeshWorkers = auditFacetedMesh

// FirstMomentsFromAuditWorkers runs the moments with an explicit worker count.
var FirstMomentsFromAuditWorkers = firstMomentsFromAudit

// PlaceMeshWorkers places a mesh with an explicit worker count.
var PlaceMeshWorkers = placeMesh

// ClosedFacetEdges validates closure and records neighbors in local edge order.
var ClosedFacetEdges = closedFacetEdges

// ClosedFacetAdjacency builds adjacency rows from the validated edge slots.
var ClosedFacetAdjacency = closedFacetAdjacency
