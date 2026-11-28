package export

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/step"
	"github.com/lestrrat-3d/step/ap214"
)

func (b *fileBuilder) addSheetFacets(ctx context.Context, vertices []r3.Vec, triangles [][3]int) ([]step.Reference, error) {
	points := make([]step.Reference, len(vertices))
	vertexRefs := make([]step.Reference, len(vertices))
	for i, p := range vertices {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		points[i] = b.add(ap214.CartesianPoint(0, "", real3(p)))
		vertexRefs[i] = b.add(ap214.VertexPoint(0, "", points[i]))
	}
	normals, err := triangleNormals(vertices, triangles)
	if err != nil {
		return nil, err
	}
	edges := make(map[edgeKey]step.Reference, len(triangles)*3/2)
	faces := make([]step.Reference, 0, len(triangles))
	for i, tri := range triangles {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		faces = append(faces, b.addTriangle(tri, normals[i], vertices, points, vertexRefs, edges))
	}
	return faces, nil
}

// addSTEPSheet writes the shell-based model shared by single parts and assemblies.
func (b *fileBuilder) addSTEPSheet(ctx context.Context, body *decad.Body, name string, plan stepPlan) (step.Reference, error) {
	var faces []step.Reference
	var err error
	if plan.analytic {
		faces, err = b.addAnalyticFaces(ctx, body)
	} else {
		faces, err = b.addSheetFacets(ctx, plan.vertices, plan.triangles)
	}
	if err != nil {
		return 0, err
	}
	groups := plan.components
	if plan.analytic {
		groups = plan.sheetFaces
	}
	faceLists := make([]step.List, len(plan.sheetClosed))
	for i, face := range faces {
		faceLists[groups[i]] = append(faceLists[groups[i]], face)
	}
	shells := make(step.List, len(faceLists))
	for i, faceList := range faceLists {
		kind := "OPEN_SHELL"
		if plan.sheetClosed[i] {
			kind = "CLOSED_SHELL"
		}
		shells[i] = b.add(step.Entity{Name: kind, Parameters: []step.Value{step.String(""), faceList}})
	}
	return b.add(step.Entity{Name: "SHELL_BASED_SURFACE_MODEL", Parameters: []step.Value{step.String(name), shells}}), nil
}

// sheetGroups maps triangles and analytic faces to the source shells and rejects mismatched mesh topology.
func sheetGroups(ctx context.Context, body *decad.Body, sources []*decad.Face, triangles [][3]int, report decad.MeshReport) ([]int, []int, []bool, error) {
	bad := func() ([]int, []int, []bool, error) {
		return nil, nil, nil, fmt.Errorf("export: STEP: %w: sheet mesh does not match its shells", decad.ErrUnsupported)
	}
	shells := body.Shells()
	if len(shells) == 0 || len(sources) != len(triangles) || report.Shells != len(shells) ||
		report.NonManifoldEdges != 0 || report.MisorientedEdges != 0 || report.DegenerateTriangles != 0 {
		return bad()
	}
	owners := make(map[*decad.Face]int)
	closed := make([]bool, len(shells))
	for i, shell := range shells {
		closed[i] = !shell.IsOpen()
		for _, face := range shell.Faces() {
			if _, exists := owners[face]; exists {
				return bad()
			}
			owners[face] = i
		}
	}
	groups := make([]int, len(triangles))
	seen := make([]bool, len(shells))
	seenFace := make(map[*decad.Face]struct{}, len(owners))
	type sheetEdge struct{ group, count int }
	edges := make(map[edgeKey]sheetEdge, len(triangles)*3/2)
	for i, tri := range triangles {
		if err := ctx.Err(); err != nil {
			return nil, nil, nil, err
		}
		group, ok := owners[sources[i]]
		if !ok {
			return bad()
		}
		groups[i] = group
		seen[group] = true
		seenFace[sources[i]] = struct{}{}
		for j := range 3 {
			key := orderedEdge(tri[j], tri[(j+1)%3])
			use, ok := edges[key]
			if ok && use.group != group {
				return bad()
			}
			use.group = group
			use.count++
			edges[key] = use
		}
	}
	open := make([]bool, len(shells))
	for _, edge := range edges {
		if edge.count == 1 {
			open[edge.group] = true
		}
	}
	for i := range shells {
		if !seen[i] || open[i] == closed[i] {
			return bad()
		}
	}
	if len(seenFace) != len(owners) {
		return bad()
	}
	faceGroups := make([]int, 0, len(owners))
	for _, face := range body.Faces() {
		group, ok := owners[face]
		if !ok {
			return bad()
		}
		faceGroups = append(faceGroups, group)
	}
	return groups, faceGroups, closed, nil
}
