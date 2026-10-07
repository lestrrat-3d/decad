package loftmesh

import (
	"context"
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/r3"
)

// LoftInput is the complete snapshot a loft restatement reads: the payload's
// held triangle set and split, its proof record, and the caller's face
// numbering. Nothing here is written.
type LoftInput struct {
	Vertices  []r3.Vec
	Triangles [][3]int // walls first, then the start cap, then the end cap
	WallCount int
	// StartCapCount is how many of Triangles[WallCount:] belong to the start
	// cap; the rest belong to the end cap.
	StartCapCount int
	// WallCell and WallSide parallel Triangles[:WallCount]: WallCell[k] is
	// {loop index, cell index}, WallSide[k] is the 0/1 half. They name the
	// side(i,j,k) role of every wall triangle.
	WallCell [][2]int
	WallSide []uint8
	// Sheet restates only the wall range and runs the sheet audits.
	Sheet bool
	// FaceOfRole maps a provenance role to the caller's face number.
	FaceOfRole map[string]int
	// StartCapRole and EndCapRole are the two cap roles a solid resolves.
	StartCapRole, EndCapRole string
	// FreeChainCounts is the recorded free boundary per face number; read
	// only when Sheet is true.
	FreeChainCounts map[int]int
	// Anchor is the orientation audit's anchor; read only when Sheet is false.
	Anchor r3.Vec
	// The payload's own proof record.
	FacetDepartureMM float64
	AreaSlackMM2     float64
	VolumeSymDiffMM3 float64
}

// LoftResult is the restated mesh in fresh storage. SourceFaces holds one
// face number per triangle.
type LoftResult struct {
	Vertices    []r3.Vec
	Triangles   [][3]int
	SourceFaces []int
	// BoundMM is every source face's bound and the mesh's own.
	BoundMM          float64
	AreaSlackMM2     float64
	VolumeSymDiffMM3 float64
	// VolumeProof is true for a solid: VolumeSymDiffMM3 is published.
	VolumeProof bool
}

// RestateLoft is docs/tessellation-reach-design.md §4's loftPayload EXACT
// RESTATEMENT (docs/tessellation-design.md §13's increment T6). A loft body
// already holds the complete, globally oriented triangle set its construction
// built and its crossing audit classified. A solid copies that set; a sheet
// copies its recorded wall range and omits the cap ranges. Both preserve the
// held wall triangles and their source faces without chording, retriangulation
// or motion. RestateLoft copies the set, names each triangle's face, publishes
// the payload's own proof and audits the result.
//
// It takes no chord tolerance, and that is the design's own reading rather
// than an omission: this path adds no chording of its own, so there is no
// chording component for a tolerance to bind, and the whole published bound is
// inherited payload displacement, which docs/tessellation-design.md §1's
// Tolerance row lets ride above tol exactly as a prism's per-end axial
// displacement does. A prism reserves its section displacement from the
// requested tolerance because its own chords are still to be chosen against
// what is left; a loft has no such choice to make, so it neither reserves nor
// refuses.
//
// Orientation needs no work here either. docs/loft-design.md §5's whole-shell
// step already turned every triangle outward from the signed tetrahedron sum,
// and placed re-runs that step on the placed triangle set, so §4's "a
// reflected placement reverses every final triangle once" rule is discharged
// by the payload and repeating it would reverse a mirrored shell twice. The
// signed orientation audit applies to the closed solid; the sheet takes its
// winding from that same whole-shell build before the caps are omitted.
//
// A solid runs closure and signed-volume orientation audits. An open sheet
// runs the manifold-with-boundary and vertex-link audits instead: the signed
// sum is anchor-dependent without caps and cannot decide orientation. A
// missing source role is Degenerate because the live topology contradicts its
// payload.
//
// A non-finite published term refuses (docs/tessellation-design.md §12): an
// absent proof must never reach a consumer as a bound. The sheet never
// publishes a volume proof, so its value does not gate sheet export. A sheet's
// area slack may include nonnegative cap allowances; they conservatively cover
// the wall-only mesh without subtracting an unproven cap contribution. The
// facet departure is published for EVERY face, and as the mesh's own bound,
// since each face's facets are exactly the payload's triangles for it. A zero
// is published only where the payload proved both facet-departure terms zero
// by value (docs/loft-design.md §5.2), which is what admits such a loft to the
// boolean's all-planar zero-bound path.
func RestateLoft(ctx context.Context, in LoftInput) (LoftResult, error) {
	if err := ctx.Err(); err != nil {
		return LoftResult{}, err
	}
	if err := requireTriangleSplit(in); err != nil {
		return LoftResult{}, err
	}

	// Provenance roles are how the payload's cells name the faces the
	// topology builder built from them (docs/evaluator-design.md §3): one
	// side(i,j,k) face per wall triangle — a loft coalesces no wall, so the
	// two halves of a cell keep their two distinct faces even when coplanar
	// (docs/tessellation-design.md §4). A sheet carries no cap role.
	faceOfRole := func(role string) (int, error) {
		f, ok := in.FaceOfRole[role]
		if !ok {
			return 0, tessellation.DegenerateError(`the body carries no face for role %q`, role)
		}
		return f, nil
	}
	var capStart, capEnd int
	if !in.Sheet {
		var err error
		capStart, err = faceOfRole(in.StartCapRole)
		if err != nil {
			return LoftResult{}, err
		}
		capEnd, err = faceOfRole(in.EndCapRole)
		if err != nil {
			return LoftResult{}, err
		}
	}

	budget := tessellation.NewAuditBudget(ctx)
	// The payload records walls first, then the start cap, then the end cap.
	// Dropping exactly the latter two ranges uses that provenance rather than
	// testing a triangle's coordinates against a section plane.
	triangles := in.Triangles
	if in.Sheet {
		triangles = in.Triangles[:in.WallCount]
	}
	src := make([]int, len(triangles))
	for k := range in.WallCount {
		if err := budget.Step(); err != nil {
			return LoftResult{}, err
		}
		f, err := faceOfRole(fmt.Sprintf("side(%d,%d,%d)", in.WallCell[k][0], in.WallCell[k][1], in.WallSide[k]))
		if err != nil {
			return LoftResult{}, err
		}
		src[k] = f
	}
	if !in.Sheet {
		for k := in.WallCount; k < in.WallCount+in.StartCapCount; k++ {
			src[k] = capStart
		}
		for k := in.WallCount + in.StartCapCount; k < len(in.Triangles); k++ {
			src[k] = capEnd
		}
	}
	if err := budget.Err(); err != nil {
		return LoftResult{}, err
	}

	if isNonFinite(in.FacetDepartureMM) || isNonFinite(in.AreaSlackMM2) ||
		(!in.Sheet && isNonFinite(in.VolumeSymDiffMM3)) {
		return LoftResult{}, tessellation.UnsupportedError(`the loft payload states no finite proof for a required mesh bound`)
	}

	// Fresh slices: the held mesh must not alias the payload's own arrays, or
	// a future consumer writing through one would rewrite the body's boundary.
	out := LoftResult{
		Vertices:     append([]r3.Vec(nil), in.Vertices...),
		Triangles:    append([][3]int(nil), triangles...),
		SourceFaces:  src,
		BoundMM:      in.FacetDepartureMM,
		AreaSlackMM2: in.AreaSlackMM2,
	}
	if !in.Sheet {
		out.VolumeSymDiffMM3 = in.VolumeSymDiffMM3
		out.VolumeProof = true
	}
	if in.Sheet {
		if err := tessellation.RequireSheetBoundary(ctx, tessellation.SheetBoundary{
			Triangles: out.Triangles, SourceFaces: out.SourceFaces, FreeChainCounts: in.FreeChainCounts,
		}); err != nil {
			return LoftResult{}, err
		}
		if err := tessellation.RequireSheetVertexLinks(ctx, len(out.Vertices), out.Triangles); err != nil {
			return LoftResult{}, err
		}
		return out, nil
	}
	if tessellation.RequireClosedMesh(out.Triangles) != nil {
		return LoftResult{}, tessellation.UnsupportedError(`the loft payload's held triangle set is not a closed mesh, so it restates no boundary`)
	}
	// docs/tessellation-design.md §4's signed-volume audit, over the same
	// identity docs/loft-design.md §5's whole-shell orientation rule reads and
	// at the same anchor the loft build used. The shell is not a void, so its
	// sum must come out positive.
	if !finiteVec(in.Anchor) {
		return LoftResult{}, tessellation.UnsupportedError(`the loft payload states no finite anchor to audit its own orientation against`)
	}
	if tessellation.OrientationSign(out.Vertices, out.Triangles, in.Anchor) <= 0 {
		return LoftResult{}, tessellation.UnsupportedError(`the loft payload's held triangle set does not enclose a positive volume, so it restates no solid`)
	}
	return out, nil
}

// requireTriangleSplit checks the payload's own wall/cap split before a
// single source face is read: the three counts index the triangle set and the
// two parallel arrays name every wall triangle's cell, so a payload disagreeing
// with itself must refuse rather than index out of its own arrays. Nothing a
// build produces reaches it — the loft build copies all five fields from one
// assembly — so it stands for a payload no evaluator wrote.
func requireTriangleSplit(in LoftInput) error {
	if len(in.Triangles) == 0 || len(in.Vertices) == 0 {
		return tessellation.DegenerateError(`the loft payload holds no triangle set to restate`)
	}
	if in.WallCount < 0 || in.StartCapCount < 0 || in.WallCount+in.StartCapCount > len(in.Triangles) {
		return tessellation.DegenerateError(`the loft payload's wall and cap triangle counts do not partition its own triangle set`)
	}
	if len(in.WallCell) != in.WallCount || len(in.WallSide) != in.WallCount {
		return tessellation.DegenerateError(`the loft payload names a cell for %d of its %d wall triangles`, len(in.WallCell), in.WallCount)
	}
	return nil
}

func isNonFinite(f float64) bool { return math.IsNaN(f) || math.IsInf(f, 0) }

// finiteVec guards exact rational lifts and every float result used by a
// certificate.
func finiteVec(v r3.Vec) bool {
	return !math.IsNaN(v.X) && !math.IsInf(v.X, 0) &&
		!math.IsNaN(v.Y) && !math.IsInf(v.Y, 0) &&
		!math.IsNaN(v.Z) && !math.IsInf(v.Z, 0)
}
