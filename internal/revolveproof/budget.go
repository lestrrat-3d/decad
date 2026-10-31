package revolveproof

import (
	"fmt"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/revolvemesh"
)

// maxFacetsPerMesh, maxFacetWorkPerCall are agent-docs/tessellation-design.md §3's
// two per-call facet ceilings, beside internal/proofbound/budget.go's proofbound.MaxFacetPairTestsPerCall.
// The facet ceilings are checked with unsigned integer arithmetic BEFORE
// allocation. Contact work is charged by the bounded candidate audit after
// assembly, before each candidate scan or exact pair test.
const (
	maxFacetsPerMesh    = 65_536
	maxFacetWorkPerCall = 262_144
)

// FacetLoops reads the builder's ring samples and axis-only walk flags without
// allocating a slice before the facet ceiling is checked.
type FacetLoops interface {
	Len() int
	Samples(i int) []revolvemesh.RevMeridian
	AxisWalk(i, walk int) bool
}

// FacetWork counts all facets charged by refinement attempts.
type FacetWork struct{ Facets uint64 }

// PreflightFacets charges agent-docs/tessellation-design.md §3's per-mesh and
// cumulative facet ceilings with unsigned integer arithmetic, BEFORE a single
// facet is allocated. work must not be nil. The cap triangles are counted from Euler's own identity
// for a polygon with holes — n + 2h − 2 triangles for n boundary samples and h
// holes — so the charge covers the whole mesh rather than its walls alone.
//
// The cumulative counters live on the call's FacetWork and are never reset by
// a refinement retry, so a sequence of attempts is charged the sum of what each
// of them asked for.
//
// sheet omits the cap charge for a PARTIAL sweep, agent-docs/surface-design.md
// §4.1's own build never triangulates one: charging it anyway would refuse a
// sheet at a finer tolerance than its own mesh actually needs. A full
// revolution already charges no cap for a solid, so sheet changes nothing
// there.
//
// The facet-contact candidate scan and its exact pair tests are charged once
// the assembled mesh exposes the spatial candidates. That work has a separate
// cumulative ceiling and does not rely on the conservative all-pairs count.
func PreflightFacets(loops FacetLoops, nPhi int, full, sheet bool, work *FacetWork) error {
	var walls, samples uint64
	for i := range loops.Len() {
		ring := loops.Samples(i)
		n := len(ring)
		for j, s := range ring {
			if loops.AxisWalk(i, s.Walk) {
				continue
			}
			per := uint64(2)
			if s.OnAxis || ring[(j+1)%n].OnAxis {
				per = 1
			}
			step, ok := MulChecked(per, uint64(nPhi))
			if !ok {
				return errRevolveFacetCeiling
			}
			walls, ok = AddChecked(walls, step)
			if !ok {
				return errRevolveFacetCeiling
			}
		}
		var ok bool
		samples, ok = AddChecked(samples, uint64(n))
		if !ok {
			return errRevolveFacetCeiling
		}
	}
	// A full revolution emits no cap at all; a partial sweep emits both,
	// unless it is a sheet, which emits neither.
	caps := uint64(0)
	if !full && !sheet && samples+2*uint64(loops.Len()) >= 4 {
		caps = 2 * (samples + 2*uint64(loops.Len()) - 4)
	}
	total, ok := AddChecked(walls, caps)
	if !ok || total > maxFacetsPerMesh {
		if ok {
			return proofbound.WithWorkLimit(errRevolveFacetCeiling, "mesh_facets", total, maxFacetsPerMesh)
		}
		return errRevolveFacetCeiling
	}
	spent, ok := AddChecked(work.Facets, total)
	if !ok || spent > maxFacetWorkPerCall {
		if ok {
			return proofbound.WithWorkLimit(errRevolveFacetCeiling, "facet_allocations", spent, maxFacetWorkPerCall)
		}
		return errRevolveFacetCeiling
	}
	work.Facets = spent
	return nil
}

var errRevolveFacetCeiling = fmt.Errorf(`%w: this chord tolerance asks for more than %d facets in one revolve mesh`, decaderr.ErrUnsupported, maxFacetsPerMesh)

// AddChecked returns a sum and whether unsigned addition stayed in range.
func AddChecked(a, b uint64) (uint64, bool) {
	sum := a + b
	return sum, sum >= a
}

// MulChecked returns a product and whether unsigned multiplication stayed in range.
func MulChecked(a, b uint64) (uint64, bool) {
	if a == 0 || b == 0 {
		return 0, true
	}
	product := a * b
	return product, product/a == b
}
