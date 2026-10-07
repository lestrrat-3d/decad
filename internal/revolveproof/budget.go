package revolveproof

import (
	"fmt"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/revolvemesh"
)

// maxFacetsPerMesh, maxFacetWorkPerCall are docs/tessellation-design.md §3's
// two per-call facet ceilings, beside internal/proofbound/budget.go's proofbound.MaxFacetPairTestsPerCall.
// Every one of them is checked with unsigned integer arithmetic BEFORE the
// allocation or audit it governs, so an over-budget request refuses rather
// than building the thing that would have blown the budget.
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

// FacetWork counts all facets and facet pairs charged by refinement attempts.
type FacetWork struct{ Facets, Pairs uint64 }

// PreflightFacets charges docs/tessellation-design.md §3's per-mesh and
// cumulative facet ceilings with unsigned integer arithmetic, BEFORE a single
// facet is allocated. work must not be nil. The cap triangles are counted from Euler's own identity
// for a polygon with holes — n + 2h − 2 triangles for n boundary samples and h
// holes — so the charge covers the whole mesh rather than its walls alone.
//
// The cumulative counters live on the call's FacetWork and are never reset by
// a refinement retry, so a sequence of attempts is charged the sum of what each
// of them asked for.
//
// sheet omits the cap charge for a PARTIAL sweep, docs/surface-design.md
// §4.1's own build never triangulates one: charging it anyway would refuse a
// sheet at a finer tolerance than its own mesh actually needs. A full
// revolution already charges no cap for a solid, so sheet changes nothing
// there.
//
// audit says whether this build will run the facet-contact audit. §3 charges
// proofbound.MaxFacetPairTestsPerCall only when it will: that ceiling bounds the work one
// all-pairs audit may do, and a build that runs no pair predicate does none of
// that work. The facet and facet-work ceilings bound ALLOCATION instead and are
// charged either way. This is why the tolerance a VerifyAll revolve refuses for
// the pair ceiling alone is met at VerifyNone — the refusal was a work budget,
// never a statement that the geometry could not be meshed.
func PreflightFacets(loops FacetLoops, nPhi int, full, sheet, audit bool, work *FacetWork) error {
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
		return errRevolveFacetCeiling
	}
	spent, ok := AddChecked(work.Facets, total)
	if !ok || spent > maxFacetWorkPerCall {
		return errRevolveFacetCeiling
	}
	charged := work.Pairs
	if audit {
		// The facet-pair audit's own ceiling, charged here rather than at the
		// audit: §3 requires the conservative F·(F−1)/2 to be checked before
		// the audit starts, and checking it before the ALLOCATION is strictly
		// earlier.
		pairs, ok := proofbound.WallChoose2(total)
		if !ok {
			return errRevolveFacetCeiling
		}
		charged, ok = AddChecked(work.Pairs, pairs)
		if !ok || charged > proofbound.MaxFacetPairTestsPerCall {
			return fmt.Errorf(`%w: this chord tolerance asks for %d facets in one revolve mesh, whose pairwise audit exceeds the fixed ceiling of %d exact tests; retry with a coarser tolerance, or ask for a mesh that does not run that audit`, decaderr.ErrUnsupported, total, proofbound.MaxFacetPairTestsPerCall)
		}
	}
	work.Facets, work.Pairs = spent, charged
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
