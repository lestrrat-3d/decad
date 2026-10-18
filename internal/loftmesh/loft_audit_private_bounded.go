package loftmesh

import (
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

const (
	privateLoftAuditMaxVertices  = 512
	privateLoftAuditMaxTriangles = 512
	privateLoftAuditMaxPairs     = 1_000_000
	privateLoftAuditMinExponent  = -32
	privateLoftAuditMaxExponent  = 32
)

// LoftCrossingAuditPrivateBounded checks a transient output shell using the
// generic audit's exact reference pair order. It is for a private caller that
// shares one 65,536-step output budget with its other proof stages. The caller
// must provide that budget; this entry does not create another allowance.
//
// All held coordinates are checked before S6's possible exact lift. For the
// admitted Frexp range, every integer reachable through the reference pair
// path stays below 2,613 bits even before common-factor stripping. An input
// outside the range is refused rather than passed to the generic audit.
// Every retained table and pair is charged before its allocation or exact
// classification. No optional sweep, grid, tree, fan, or candidate list is
// built, so the first failing pair remains the reference path's first pair.
func LoftCrossingAuditPrivateBounded(budget *proofbound.WorkBudget,
	verts []r3.Vec, tris [][3]int) error {
	if err := budget.Err(); err != nil {
		return err
	}
	v, f := len(verts), len(tris)
	if v > privateLoftAuditMaxVertices || f > privateLoftAuditMaxTriangles {
		return fmt.Errorf("%w: private loft crossing shell exceeds 512 vertices or facets", decaderr.ErrUnsupported)
	}
	for _, vertex := range verts {
		if err := budget.Step(); err != nil {
			return err
		}
		for _, coordinate := range [...]float64{vertex.X, vertex.Y, vertex.Z} {
			if coordinate == 0 {
				continue
			}
			if math.IsNaN(coordinate) || math.IsInf(coordinate, 0) {
				return fmt.Errorf("%w: private loft crossing has a nonfinite vertex", decaderr.ErrUnsupported)
			}
			_, exponent := math.Frexp(coordinate)
			if exponent < privateLoftAuditMinExponent || exponent > privateLoftAuditMaxExponent {
				return fmt.Errorf("%w: private loft crossing vertex exceeds the exact-bit gate", decaderr.ErrUnsupported)
			}
		}
	}
	for _, tri := range tris {
		if err := budget.Step(); err != nil {
			return err
		}
		for _, index := range tri {
			if index < 0 || index >= v {
				return fmt.Errorf("%w: private loft crossing facet has an invalid vertex", decaderr.ErrUnsupported)
			}
		}
	}
	if err := budget.Err(); err != nil {
		return err
	}

	// S6 must follow the coordinate gate: TriangleCollapsed can take an exact
	// XptOf/Xsub/Xcross fallback before any audit table exists.
	for i, tri := range tris {
		if err := budget.Step(); err != nil {
			return err
		}
		if TriangleCollapsed(verts, tri) {
			return fmt.Errorf("%w: loft triangle %d has collapsed to zero area", decaderr.ErrDegenerate, i)
		}
	}
	if err := budget.Err(); err != nil {
		return err
	}

	pairs := uint64(f * (f - 1) / 2)
	if pairs > privateLoftAuditMaxPairs {
		return errLoftAuditCeiling()
	}
	// The private table holds one vertex lift and seven facet-sized slices;
	// each facet also owns three projected corners. Reserve their logical
	// entries and per-entry exact construction before allocating the slices.
	// The extra slots cover the bounded integer and rational values built by
	// those entries. Pair scratch is bounded and discarded after each pair.
	reserve := uint64(8*v + 32*f)
	if _, err := budget.StepN(reserve); err != nil {
		return err
	}
	if err := budget.Err(); err != nil {
		return err
	}
	data := &LoftAuditData{
		Xverts:      make([]proof.Xpt, v),
		Corners:     make([][3]r3.Vec, f),
		Xtris:       make([][3]proof.Xpt, f),
		Norms:       make([]proof.Xpt, f),
		floatNorms:  make([]r3.Vec, f),
		Planes:      make([]LoftExactPlane, f),
		Projections: make([][3]meshbool.Xp2, f),
		tris:        make([][3]int, f),
	}
	copy(data.tris, tris)
	for i, vertex := range verts {
		if err := budget.Step(); err != nil {
			return err
		}
		data.Xverts[i] = proof.XptOf(vertex)
	}
	for i, tri := range tris {
		if err := budget.Step(); err != nil {
			return err
		}
		corners := LoftTriCorners(verts, tri)
		xtri := [3]proof.Xpt{
			data.Xverts[tri[0]], data.Xverts[tri[1]], data.Xverts[tri[2]],
		}
		normal := proof.Xcross(proof.Xsub(xtri[1], xtri[0]), proof.Xsub(xtri[2], xtri[0]))
		data.Corners[i], data.Xtris[i], data.Norms[i] = corners, xtri, normal
		data.floatNorms[i] = normal.Vec()
		data.Planes[i] = NewLoftExactPlane(xtri[0], normal)
		u, w := meshbool.ProjAxes(normal)
		for k, point := range xtri {
			if err := budget.Step(); err != nil {
				return err
			}
			data.Projections[i][k] = meshbool.NewXP2FromXpt(point, u, w)
		}
	}
	if err := budget.Err(); err != nil {
		return err
	}

	for i := range tris {
		for j := i + 1; j < f; j++ {
			if err := budget.Step(); err != nil {
				return err
			}
			if _, err := AuditLoftPairData(data, tris, i, j, LoftAuditShortcuts{}); err != nil {
				return err
			}
		}
	}
	return budget.Err()
}
