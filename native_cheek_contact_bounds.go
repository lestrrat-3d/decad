package decad

import (
	"context"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/meshbool"

	"github.com/lestrrat-3d/decad/internal/loftrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/rimextension"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
)

// contactBounds registers a convex cylindrical band ending at the native top
// cap. Every denoted tool point lies within the tested tool's bounding box
// expanded by its original two-sided facet bound. The lower station includes
// the target's original uncertainty so any target counterpart near this tool
// falls within the chart, in either direction. The true face ends at the cap.
func (s *cheekCutSource) contactBounds(ctx context.Context, stock, tool *topo.Mesh, m *proofbound.ReplayBudget) (map[int]meshbool.CutFacetBound, bool, error) {
	if len(stock.Triangles) == 0 || len(stock.Triangles) > 65536 || len(tool.Triangles) == 0 || len(tool.Triangles) > 8192 {
		return nil, false, nil
	}
	targetBounds, err := topobuild.ReplayVertexBounds(ctx, stock, m)
	if err != nil {
		return nil, false, err
	}
	toolBounds, err := topobuild.ReplayVertexBounds(ctx, tool, m)
	if err != nil {
		return nil, false, err
	}
	if len(targetBounds) != len(stock.Vertices) || len(toolBounds) != len(tool.Vertices) {
		return nil, false, nil
	}
	if err := m.ReserveBundle(ctx, uint64(len(targetBounds)+len(toolBounds)+len(stock.Source)), uint64(len(stock.Source)), uint64(len(stock.Source))*32); err != nil {
		return nil, false, err
	}
	beta := 0.0
	for _, b := range targetBounds {
		if b < 0 || proofbound.IsNonFinite(b) {
			return nil, false, nil
		}
		beta = max(beta, b)
	}
	minY := math.Inf(1)
	toolBeta := 0.0
	for i, v := range tool.Vertices {
		b := toolBounds[i]
		if b < 0 || proofbound.IsNonFinite(b) || !proofbound.FiniteVec(v) {
			return nil, false, nil
		}
		minY = min(minY, v.Y)
		toolBeta = max(toolBeta, b)
	}
	lower := math.Nextafter(math.Nextafter(minY-toolBeta, math.Inf(-1))-beta, math.Inf(-1))
	if proofbound.IsNonFinite(lower) {
		return nil, false, nil
	}
	source, ok := s.tool.n.Payload.(loftrecord.Payload)
	if !ok {
		return nil, false, nil
	}
	start, end := source.Frame0.Origin(), source.Frame1.Origin()
	if start.X != end.X || start.Z != end.Z {
		return nil, false, nil
	}
	originX, originZ := new(big.Rat).SetFloat64(start.X), new(big.Rat).SetFloat64(start.Z)
	if originX == nil || originZ == nil {
		return nil, false, nil
	}
	low := new(big.Rat).SetFloat64(lower)
	seen := map[*topo.Face]bool{}
	for _, face := range stock.Source {
		if seen[face] || len(face.Origins) != 1 || face.Origins[0].Producer != s.root.alignedCheek.Origin.Producer {
			continue
		}
		seen[face] = true
		f, ok, err := s.reader.bind(ctx, s.target, face, false)
		if err != nil {
			return nil, false, err
		}
		if !ok || f.Body != s.root.alignedCheek {
			continue
		}
		n, ok, err := rimextension.ReadPolarSource(ctx, f, m)
		if err != nil {
			return nil, false, err
		}
		if !ok || n.CenterX.Cmp(originX) != 0 || n.CenterZ.Cmp(originZ) != 0 {
			continue
		}
		top, _ := n.Top.Float64()
		if lower >= top || low.Cmp(n.Bottom) <= 0 {
			continue
		}
		bounds, ok, err := s.registeredBand(ctx, stock, targetBounds, f, n, low, new(big.Rat).SetFloat64(top), m)
		if err != nil {
			return nil, false, err
		}
		if ok {
			return bounds, true, nil
		}
	}
	return nil, false, nil
}
