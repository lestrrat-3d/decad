package booleaneval

import (
	"context"
	"math"
	"math/big"
	"slices"
	"unsafe"

	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/surfacenormal"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// snapshotStoredShift binds an accepted Cut to its ordered operands and makes
// private copies charged to the retry's shared budget.
func snapshotStoredShift(ctx context.Context, cert *ShiftCertificate, result, target, tool *topo.Body, budget *proofbound.ReplayBudget) (*ShiftCertificate, error) {
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	if budget == nil || !budget.IsBrakeReplay() {
		return nil, errShiftSnapshotMissingBrakeReplayBudget
	}
	if err := budget.SpendWork(ctx, 64); err != nil {
		return nil, err
	}
	if cert == nil ||
		target == nil ||
		tool == nil ||
		!cert.sealed ||
		(result != nil &&
			(result.PrivateShift != cert ||
				cert.resultID != result.Origin.Producer)) ||
		cert.target != target ||
		cert.tool != tool ||
		cert.targetID != target.Origin.Producer ||
		cert.toolID != tool.Origin.Producer ||
		(!cert.IsTranslation() &&
			!cert.IsPureGrowth()) {
		return nil, errShiftSnapshotStoredShiftIdentityMismatch
	}
	if !proofbound.FiniteVec(cert.shift.Move) || !proofbound.FiniteVec(cert.center) {
		return nil, errShiftSnapshotNonfiniteStoredShift
	}
	for _, v := range []float64{cert.targetChord, cert.radius, cert.factor, cert.chord, cert.scale, cert.displacement, cert.reach, cert.shift.Grow} { //nolint:modernize // Check stored scalars in their recorded order.
		if proofbound.IsNonFinite(v) {
			return nil, errShiftSnapshotNonfiniteStoredShift
		}
	}
	if err := budget.ReserveRecords(ctx, 1); err != nil {
		return nil, err
	}
	if err := budget.ReserveArray(ctx, 1, uint64(unsafe.Sizeof(ShiftCertificate{}))); err != nil {
		return nil, err
	}
	out := *cert
	var err error
	for _, item := range []struct {
		source *topo.Mesh
		dest   **topo.Mesh
	}{{cert.selected, &out.selected}, {cert.decision, &out.decision}, {cert.charged, &out.charged}, {cert.targetMesh, &out.targetMesh}} {
		if item.source == nil {
			return nil, errShiftSnapshotStoredShiftMeshMissing
		}
		*item.dest, err = snapshotShiftMesh(ctx, item.source, budget)
		if err != nil {
			return nil, err
		}
	}
	out.component.ofVertex, err = snapshotShiftSlice(ctx, cert.component.ofVertex, budget)
	if err != nil {
		return nil, err
	}
	out.component.ofFacet, err = snapshotShiftSlice(ctx, cert.component.ofFacet, budget)
	if err != nil {
		return nil, err
	}
	if cert.component.count != 1 || len(out.component.ofVertex) != len(out.selected.Vertices) || len(out.component.ofFacet) != len(out.selected.Triangles) {
		return nil, errShiftSnapshotStoredShiftPiecesMismatch
	}
	if err := budget.SpendWork(ctx, uint64(len(out.component.ofVertex))+uint64(len(out.component.ofFacet))); err != nil {
		return nil, err
	}
	for _, n := range out.component.ofVertex {
		if n != 0 {
			return nil, errShiftSnapshotStoredShiftPieceMismatch
		}
	}
	for _, n := range out.component.ofFacet {
		if n != 0 {
			return nil, errShiftSnapshotStoredShiftPieceMismatch
		}
	}
	out.groups, err = snapshotShiftGroups(ctx, cert.groups, budget)
	if err != nil {
		return nil, err
	}
	return &out, nil
}

func snapshotShiftGroups(ctx context.Context, source []facetedrecord.Group, budget *proofbound.ReplayBudget) ([]facetedrecord.Group, error) {
	groups, err := snapshotShiftSlice(ctx, source, budget)
	if err != nil {
		return nil, err
	}
	if uint64(len(groups)) > ^uint64(0)/64 {
		return nil, errShiftSnapshotTooManyGroups
	}
	if err := budget.SpendWork(ctx, uint64(len(groups))*64); err != nil {
		return nil, err
	}
	for i := range groups {
		g := &groups[i]
		if g.Denoted != nil {
			g.Denoted, err = snapshotDenoted(ctx, g.Denoted, budget)
			if err != nil {
				return nil, err
			}
		}
		switch g.Surface.(type) {
		case nil, surfacegeom.Plane, surfacegeom.Cylinder, surfacegeom.Cone, surfacegeom.Sphere, surfacegeom.Torus, surfacegeom.NURBSSurface, surfacegeom.Faceted:
		default:
			return nil, errShiftSnapshotStoredSurfaceSnapshotUnavailable
		}
		g.Origins, err = snapshotShiftSlice(ctx, g.Origins, budget)
		if err != nil {
			return nil, err
		}
		g.Upstream, err = snapshotShiftSlice(ctx, g.Upstream, budget)
		if err != nil {
			return nil, err
		}
		if g.CapRecord != nil {
			src := g.CapRecord
			if err := budget.ReserveRecords(ctx, 1); err != nil {
				return nil, err
			}
			if err := budget.ReserveArray(ctx, 1, uint64(unsafe.Sizeof(topo.CapRecord{}))); err != nil {
				return nil, err
			}
			c := *src
			if src.ZDenoted != nil {
				n, d := src.ZDenoted.Num().BitLen(), src.ZDenoted.Denom().BitLen()
				if n > 4096 || d > 4096 || src.ZDenoted.Denom().Sign() <= 0 {
					return nil, errShiftSnapshotStoredCapRationalWidth
				}
				words := uint64((n+63)/64 + (d+63)/64 + 2)
				if err := budget.SpendWork(ctx, 2*words+1); err != nil {
					return nil, err
				}
				if err := budget.ReserveRecords(ctx, 3); err != nil {
					return nil, err
				}
				if err := budget.ReserveArray(ctx, words, 8); err != nil {
					return nil, err
				}
				if err := budget.ReserveArray(ctx, 1, uint64(unsafe.Sizeof(big.Rat{}))); err != nil {
					return nil, err
				}
				c.ZDenoted = new(big.Rat).Set(src.ZDenoted)
			}
			g.CapRecord = &c
		}
	}
	return groups, nil
}

func snapshotDenoted(ctx context.Context, source *surfacenormal.Revolved, budget *proofbound.ReplayBudget) (*surfacenormal.Revolved, error) {
	if err := budget.SpendWork(ctx, 1); err != nil {
		return nil, err
	}
	if err := budget.ReserveRecords(ctx, 1); err != nil {
		return nil, err
	}
	if err := budget.ReserveArray(ctx, 1, uint64(unsafe.Sizeof(surfacenormal.Revolved{}))); err != nil {
		return nil, err
	}
	out := *source
	var err error
	out.Runs, err = snapshotShiftSlice(ctx, source.Runs, budget)
	if err != nil {
		return nil, err
	}
	out.Ends, err = snapshotShiftSlice(ctx, source.Ends, budget)
	if err != nil {
		return nil, err
	}
	clone := func(x *proofbound.RatInterval) error {
		var e error
		x.Lo, e = snapshotDenotedRat(ctx, x.Lo, budget)
		if e != nil {
			return e
		}
		x.Hi, e = snapshotDenotedRat(ctx, x.Hi, budget)
		return e
	}
	for i := range out.Origin {
		if err = clone(&out.Origin[i]); err != nil {
			return nil, err
		}
	}
	for i := range out.Basis {
		for j := range out.Basis[i] {
			if err = clone(&out.Basis[i][j]); err != nil {
				return nil, err
			}
		}
	}
	for i := range out.Runs {
		for j := range out.Runs[i] {
			if err = clone(&out.Runs[i][j]); err != nil {
				return nil, err
			}
		}
	}
	for i := range out.Ends {
		for j := range out.Ends[i] {
			for k := range out.Ends[i][j] {
				if err = clone(&out.Ends[i][j][k]); err != nil {
					return nil, err
				}
			}
		}
	}
	for i := range out.Centre {
		if err = clone(&out.Centre[i]); err != nil {
			return nil, err
		}
	}
	if err = clone(&out.Radius); err != nil {
		return nil, err
	}
	for i := range out.CapDir {
		if err = clone(&out.CapDir[i]); err != nil {
			return nil, err
		}
	}
	return &out, nil
}

func snapshotDenotedRat(ctx context.Context, source *big.Rat, budget *proofbound.ReplayBudget) (*big.Rat, error) {
	if source == nil {
		return nil, nil //nolint:nilnil // A nil input is an absent optional value.
	}
	n, d := source.Num().BitLen(), source.Denom().BitLen()
	if n > 4096 || d > 4096 || source.Denom().Sign() <= 0 {
		return nil, errShiftSnapshotDenotationWidth
	}
	words := uint64((n+63)/64 + (d+63)/64 + 2)
	if err := budget.SpendWork(ctx, words*2+1); err != nil {
		return nil, err
	}
	if err := budget.ReserveRecords(ctx, 3); err != nil {
		return nil, err
	}
	if err := budget.ReserveArray(ctx, words, 8); err != nil {
		return nil, err
	}
	if err := budget.ReserveArray(ctx, 1, uint64(unsafe.Sizeof(big.Rat{}))); err != nil {
		return nil, err
	}
	return new(big.Rat).Set(source), nil
}

func snapshotShiftSlice[T any](ctx context.Context, src []T, budget *proofbound.ReplayBudget) ([]T, error) {
	if src == nil {
		return nil, nil
	}
	n := uint64(len(src))
	if n > proofbound.BrakeReplayRecords {
		return nil, errShiftSnapshotStoredSliceTooLarge
	}
	if err := budget.SpendWork(ctx, n); err != nil {
		return nil, err
	}
	if err := budget.ReserveRecords(ctx, n); err != nil {
		return nil, err
	}
	var x T
	if err := budget.ReserveArray(ctx, n, uint64(unsafe.Sizeof(x))); err != nil {
		return nil, err
	}
	return slices.Clone(src), nil
}

func snapshotShiftMesh(ctx context.Context, src *topo.Mesh, budget *proofbound.ReplayBudget) (*topo.Mesh, error) {
	if src == nil || budget == nil {
		return nil, errShiftSnapshotMissingStoredMesh
	}
	if err := budget.SpendWork(ctx, 16); err != nil {
		return nil, err
	}
	for _, v := range []float64{src.Bound, src.CoordBound, src.AreaSlack, src.VolSymDiff} {
		if v < 0 || proofbound.IsNonFinite(v) {
			return nil, errShiftSnapshotStoredMeshBoundInvalid
		}
	}
	if len(src.Vertices) > 65536 ||
		len(src.Triangles) > 65536 ||
		len(src.VertexBound) > 65536 ||
		len(src.Source) > 65536 ||
		src.ReplayFaceBoundCount() > 65536 ||
		len(src.Source) != len(src.Triangles) ||
		(src.VertexBound != nil &&
			len(src.VertexBound) != len(src.Vertices)) {
		return nil, errShiftSnapshotStoredMeshShape
	}
	if err := budget.ReserveRecords(ctx, 1); err != nil {
		return nil, err
	}
	if err := budget.ReserveArray(ctx, 1, uint64(unsafe.Sizeof(topo.Mesh{}))); err != nil {
		return nil, err
	}
	out := *src
	out.Owner = nil
	var err error
	out.Vertices, err = snapshotShiftSlice[r3.Vec](ctx, src.Vertices, budget)
	if err != nil {
		return nil, err
	}
	out.Triangles, err = snapshotShiftSlice[[3]int](ctx, src.Triangles, budget)
	if err != nil {
		return nil, err
	}
	out.VertexBound, err = snapshotShiftSlice[float64](ctx, src.VertexBound, budget)
	if err != nil {
		return nil, err
	}
	out.Source, err = snapshotShiftSlice[*topo.Face](ctx, src.Source, budget)
	if err != nil {
		return nil, err
	}
	if err := budget.ReserveRecords(ctx, 1); err != nil {
		return nil, err
	}
	if err := budget.ReserveArray(ctx, 1, 256); err != nil {
		return nil, err
	}
	table, err := topo.NewFaceBoundTable(ctx, src.ReplayFaceBoundCount(), budget)
	if err != nil {
		return nil, err
	}
	if err := src.EachReplayFaceBound(ctx, budget, func(f *topo.Face, v float64) error {
		if f == nil || math.IsNaN(v) || math.IsInf(v, 0) || v < 0 {
			return errShiftSnapshotStoredFaceBoundInvalid
		}
		return table.Set(ctx, f, v, budget)
	}); err != nil {
		return nil, err
	}
	out.FaceBound = nil
	out.ReplayFaceBounds = table
	if err := budget.SpendWork(ctx, uint64(len(out.Vertices))+uint64(len(out.Triangles))+1); err != nil {
		return nil, err
	}
	for i, v := range out.Vertices {
		if !proofbound.FiniteVec(v) || (out.VertexBound != nil && (proofbound.IsNonFinite(out.VertexBound[i]) || out.VertexBound[i] < 0)) {
			return nil, errShiftSnapshotStoredMeshVertexInvalid
		}
	}
	for i, tri := range out.Triangles {
		if out.Source[i] == nil {
			return nil, errShiftSnapshotStoredMeshSourceMissing
		}
		for _, v := range tri {
			if v < 0 || v >= len(out.Vertices) {
				return nil, errShiftSnapshotStoredMeshIndexInvalid
			}
		}
		if _, ok, err := out.ReplayFaceBound(ctx, out.Source[i], budget); err != nil {
			return nil, err
		} else if !ok {
			return nil, errShiftSnapshotStoredMeshFaceBoundMissing
		}
	}
	return &out, nil
}

type shiftSourceSeal struct {
	ptr   *provenance.SourcePatch
	value provenance.SourcePatch
}

type shiftGroupPointers struct {
	cap     *topo.CapRecord
	level   *big.Rat
	denoted *surfacenormal.Revolved
}

// ShiftCertificateSnapshot binds one private copy to its source and shared meter.
type ShiftCertificateSnapshot struct {
	meshes        [4]*topo.Mesh
	groupPointers []shiftGroupPointers
	sources       []shiftSourceSeal
	source        *ShiftCertificate
	copy          *ShiftCertificate
	budget        *proofbound.ReplayBudget
}

// SnapshotShiftCertificate copies the stored evidence before retry processing.
func SnapshotShiftCertificate(ctx context.Context, budget *proofbound.ReplayBudget, cert *ShiftCertificate) (*ShiftCertificateSnapshot, error) {
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	if budget == nil || !budget.IsBrakeReplay() {
		return nil, errShiftSnapshotMissingBrakeReplayBudget
	}
	if err := budget.SpendWork(ctx, 2); err != nil {
		return nil, err
	}
	if cert == nil {
		return nil, errShiftSnapshotMissingStoredShift
	}
	if err := validateShiftCaptureSeal(ctx, cert, budget); err != nil {
		return nil, err
	}
	snapshotCopy, err := snapshotStoredShift(ctx, cert, nil, cert.target, cert.tool, budget)
	if err != nil {
		return nil, err
	}
	if err := budget.ReserveRecords(ctx, 1); err != nil {
		return nil, err
	}
	if err := budget.ReserveArray(ctx, 1, uint64(unsafe.Sizeof(ShiftCertificateSnapshot{}))); err != nil {
		return nil, err
	}
	if err := budget.ReserveRecords(ctx, uint64(len(cert.groups))); err != nil {
		return nil, err
	}
	if err := budget.ReserveArray(ctx, uint64(len(cert.groups)), uint64(unsafe.Sizeof(shiftGroupPointers{}))); err != nil {
		return nil, err
	}
	if err := budget.SpendWork(ctx, uint64(len(cert.groups))); err != nil {
		return nil, err
	}
	pointers := make([]shiftGroupPointers, len(cert.groups))
	for i, g := range cert.groups {
		pointers[i].cap = g.CapRecord
		pointers[i].denoted = g.Denoted
		if g.CapRecord != nil {
			pointers[i].level = g.CapRecord.ZDenoted
		}
	}
	var count uint64
	for _, g := range cert.groups {
		p := g.Source
		for depth := 0; p != nil; depth++ {
			if depth >= 64 {
				return nil, errShiftSnapshotSourceAncestryTooDeep
			}
			if err := budget.SpendWork(ctx, 1); err != nil {
				return nil, err
			}
			count++
			p = p.Parent
		}
	}
	if err := budget.ReserveRecords(ctx, count); err != nil {
		return nil, err
	}
	if err := budget.ReserveArray(ctx, count, uint64(unsafe.Sizeof(shiftSourceSeal{}))); err != nil {
		return nil, err
	}
	if err := budget.SpendWork(ctx, count); err != nil {
		return nil, err
	}
	seals := make([]shiftSourceSeal, 0, count)
	for _, g := range cert.groups {
		for p := g.Source; p != nil; p = p.Parent {
			seals = append(seals, shiftSourceSeal{ptr: p, value: *p})
		}
	}
	return &ShiftCertificateSnapshot{source: cert, copy: snapshotCopy, budget: budget, sources: seals, groupPointers: pointers, meshes: [4]*topo.Mesh{cert.selected, cert.decision, cert.charged, cert.targetMesh}}, nil
}

// Revalidate checks the committed result, ordered operands and copied fields.
func (s *ShiftCertificateSnapshot) Revalidate(ctx context.Context, budget *proofbound.ReplayBudget, result, target, tool *topo.Body) error {
	if s == nil || budget == nil || s.budget != budget {
		return errShiftSnapshotMismatchedSnapshotBudget
	}
	if [4]*topo.Mesh{s.source.selected, s.source.decision, s.source.charged, s.source.targetMesh} != s.meshes {
		return errShiftSnapshotStoredMeshPointerChanged
	}
	if len(s.source.groups) != len(s.groupPointers) {
		return errShiftSnapshotSourceGroupsChanged
	}
	if err := budget.SpendWork(ctx, uint64(len(s.groupPointers))); err != nil {
		return err
	}
	for i, p := range s.groupPointers {
		g := s.source.groups[i]
		if g.CapRecord != p.cap || g.Denoted != p.denoted || (g.CapRecord != nil && g.CapRecord.ZDenoted != p.level) {
			return errShiftSnapshotSourceGroupPointerChanged
		}
	}
	if err := budget.SpendWork(ctx, uint64(len(s.sources))); err != nil {
		return err
	}
	for _, x := range s.sources {
		if x.ptr == nil || *x.ptr != x.value {
			return errShiftSnapshotSourceAncestryChanged
		}
	}
	return revalidateShiftSnapshot(ctx, s.source, s.copy, result, target, tool, budget)
}

// Certificate revalidates and returns the private copied evidence for this retry.
func (s *ShiftCertificateSnapshot) Certificate(ctx context.Context, budget *proofbound.ReplayBudget, result, target, tool *topo.Body) (*ShiftCertificate, error) {
	if err := s.Revalidate(ctx, budget, result, target, tool); err != nil {
		return nil, err
	}
	return s.copy, nil
}

// revalidateShiftSnapshot compares the live stored certificate with its private
// copy immediately before use. Both calls must use the same replay budget.
func revalidateShiftSnapshot(ctx context.Context, source, snapshot *ShiftCertificate, result, target, tool *topo.Body, budget *proofbound.ReplayBudget) error {
	if err := ctx.Err(); err != nil {
		return err
	}
	if source == nil ||
		snapshot == nil ||
		budget == nil ||
		!budget.IsBrakeReplay() ||
		result == nil ||
		target == nil ||
		tool == nil ||
		result.PrivateShift != source ||
		!source.sealed ||
		source.target != target ||
		source.tool != tool ||
		source.targetID != target.Origin.Producer ||
		source.toolID != tool.Origin.Producer ||
		source.resultID != result.Origin.Producer {
		return errShiftSnapshotStaleStoredShift
	}
	if err := budget.SpendWork(ctx, 512); err != nil {
		return err
	}
	if !sameShiftScalars(source, snapshot) {
		return errShiftSnapshotStoredShiftFieldsChanged
	}
	for _, pair := range [][2]*topo.Mesh{{source.selected, snapshot.selected}, {source.decision, snapshot.decision}, {source.charged, snapshot.charged}, {source.targetMesh, snapshot.targetMesh}} {
		a, b := pair[0], pair[1]
		if a == nil || b == nil || len(a.Vertices) != len(b.Vertices) || len(a.Triangles) != len(b.Triangles) || len(a.VertexBound) != len(b.VertexBound) || len(a.Source) != len(b.Source) {
			return errShiftSnapshotStoredMeshChanged
		}
		count := uint64(len(a.Vertices)) + uint64(len(a.Triangles)) + uint64(len(a.VertexBound)) + uint64(len(a.Source)) + uint64(len(a.FaceBound)) + 1
		if count > ^uint64(0)/64 {
			return errShiftSnapshotStoredMeshTooLarge
		}
		if err := budget.SpendWork(ctx, count*64); err != nil {
			return err
		}
		if !sameSnapshotMesh(a, b) {
			return errShiftSnapshotStoredMeshChanged
		}
		if a.ReplayFaceBoundCount() != b.ReplayFaceBoundCount() {
			return errShiftSnapshotStoredBoundsChanged
		}
		if err := a.EachReplayFaceBound(ctx, budget, func(f *topo.Face, v float64) error {
			got, ok, e := b.ReplayFaceBound(ctx, f, budget)
			if e != nil {
				return e
			}
			if !ok || !sameFloat(v, got) {
				return errShiftSnapshotStoredBoundsChanged
			}
			return nil
		}); err != nil {
			return err
		}
	}
	if len(source.groups) != len(snapshot.groups) || len(source.component.ofVertex) != len(snapshot.component.ofVertex) || len(source.component.ofFacet) != len(snapshot.component.ofFacet) {
		return errShiftSnapshotStoredGroupsChanged
	}
	count := uint64(len(source.groups)) + uint64(len(source.component.ofVertex)) + uint64(len(source.component.ofFacet)) + 1
	for i, g := range source.groups {
		other := snapshot.groups[i]
		if len(g.Origins) != len(other.Origins) || len(g.Upstream) != len(other.Upstream) {
			return errShiftSnapshotStoredProvenanceChanged
		}
		for _, n := range []int{len(g.Origins), len(g.Upstream)} {
			if uint64(n) > ^uint64(0)-count {
				return errShiftSnapshotStoredProvenanceTooLarge
			}
			count += uint64(n)
		}
	}
	if count > ^uint64(0)/512 {
		return errShiftSnapshotStoredGroupsTooLarge
	}
	if err := budget.SpendWork(ctx, count*512); err != nil {
		return err
	}
	if (source.component.ofVertex == nil) != (snapshot.component.ofVertex == nil) ||
		(source.component.ofFacet == nil) != (snapshot.component.ofFacet == nil) ||
		!slices.Equal(source.component.ofVertex, snapshot.component.ofVertex) ||
		!slices.Equal(source.component.ofFacet, snapshot.component.ofFacet) {
		return errShiftSnapshotStoredComponentsChanged
	}
	for i := range source.groups {
		a, b := source.groups[i], snapshot.groups[i]
		if (a.Denoted == nil) != (b.Denoted == nil) {
			return errShiftSnapshotDenotationChanged
		}
		if a.Denoted != nil {
			if len(a.Denoted.Runs) != len(b.Denoted.Runs) || len(a.Denoted.Ends) != len(b.Denoted.Ends) {
				return errShiftSnapshotDenotationChanged
			}
			n := uint64(len(a.Denoted.Runs)) + uint64(len(a.Denoted.Ends)) + 16
			if n > proofbound.BrakeReplayRecords || n > ^uint64(0)/4096 {
				return errShiftSnapshotDenotationTooLarge
			}
			if err := budget.SpendWork(ctx, n*4096); err != nil {
				return err
			}
		}
		ca, cb := a.CapRecord, b.CapRecord
		a.CapRecord = nil
		b.CapRecord = nil
		if !sameShiftGroup(a, b) || (ca == nil) != (cb == nil) {
			return errShiftSnapshotStoredGroupChanged
		}
		if ca != nil {
			xa, xb := *ca, *cb
			ra, rb := xa.ZDenoted, xb.ZDenoted
			xa.ZDenoted = nil
			xb.ZDenoted = nil
			if !sameFrame(xa.Frame, xb.Frame) || !sameTransform(xa.Xform, xb.Xform) || !sameFloat(xa.Z, xb.Z) || (ra == nil) != (rb == nil) {
				return errShiftSnapshotStoredCapChanged
			}
			if ra != nil {
				if ra.Num().BitLen() > 4096 || ra.Denom().BitLen() > 4096 || rb.Num().BitLen() > 4096 || rb.Denom().BitLen() > 4096 {
					return errShiftSnapshotStoredCapWidth
				}
				if err := budget.SpendWork(ctx, 256); err != nil {
					return err
				}
				if ra.Num().Cmp(rb.Num()) != 0 || ra.Denom().Cmp(rb.Denom()) != 0 {
					return errShiftSnapshotStoredCapChanged
				}
			}
		}
	}
	return nil
}

func sameFloat(a, b float64) bool { return math.Float64bits(a) == math.Float64bits(b) }
func sameVec(a, b r3.Vec) bool {
	return sameFloat(a.X, b.X) && sameFloat(a.Y, b.Y) && sameFloat(a.Z, b.Z)
}
func sameFrame(a, b r3.Frame) bool {
	return sameVec(a.Origin(), b.Origin()) && sameVec(a.U(), b.U()) && sameVec(a.V(), b.V()) && sameVec(a.N(), b.N())
}
func sameTransform(a, b r3.Transform) bool {
	x, y := a.Basis(), b.Basis()
	return sameVec(x.EX, y.EX) && sameVec(x.EY, y.EY) && sameVec(x.EZ, y.EZ) && sameVec(a.Translation(), b.Translation())
}
func sameValue(a, b units.Value) bool { return a.Unit() == b.Unit() && sameFloat(a.Mag(), b.Mag()) }
func sameShiftScalars(a, b *ShiftCertificate) bool {
	x, y := a.publicWarning, b.publicWarning
	return a.target == b.target &&
		a.tool == b.tool &&
		a.targetID == b.targetID &&
		a.toolID == b.toolID &&
		a.resultID == b.resultID &&
		a.sealed == b.sealed &&
		a.component.count == b.component.count &&
		sameVec(a.center, b.center) &&
		sameVec(a.shift.Move, b.shift.Move) &&
		sameFloat(a.shift.Grow, b.shift.Grow) &&
		sameFloat(a.targetChord, b.targetChord) &&
		sameFloat(a.radius, b.radius) &&
		sameFloat(a.factor, b.factor) &&
		sameFloat(a.chord, b.chord) &&
		sameFloat(a.scale, b.scale) &&
		sameFloat(a.displacement, b.displacement) &&
		sameFloat(a.reach, b.reach) &&
		x.Operation == y.Operation &&
		x.PublicOperation == y.PublicOperation &&
		x.Code == y.Code &&
		x.Operand == y.Operand &&
		x.Change == y.Change &&
		x.AutomaticAllowance == y.AutomaticAllowance &&
		x.ChainAllowance == y.ChainAllowance &&
		sameFloat(x.DisplacementBound, y.DisplacementBound) &&
		sameFloat(x.AllowedReach, y.AllowedReach) &&
		sameFloat(x.TouchedBound, y.TouchedBound) &&
		sameFloat(x.PairChordLimit, y.PairChordLimit)
}
func sameSnapshotMesh(a, b *topo.Mesh) bool {
	if !sameFloat(a.Bound, b.Bound) ||
		!sameFloat(a.CoordBound, b.CoordBound) ||
		!sameFloat(a.AreaSlack, b.AreaSlack) ||
		!sameFloat(a.VolSymDiff, b.VolSymDiff) ||
		a.SymDiffOK != b.SymDiffOK ||
		a.BoundaryOK != b.BoundaryOK ||
		(a.Vertices == nil) != (b.Vertices == nil) ||
		(a.Triangles == nil) != (b.Triangles == nil) ||
		(a.Source == nil) != (b.Source == nil) ||
		(a.VertexBound == nil) != (b.VertexBound == nil) {
		return false
	}
	for i := range a.Vertices {
		if !sameVec(a.Vertices[i], b.Vertices[i]) {
			return false
		}
	}
	for i := range a.VertexBound {
		if !sameFloat(a.VertexBound[i], b.VertexBound[i]) {
			return false
		}
	}
	return slices.Equal(a.Triangles, b.Triangles) && slices.Equal(a.Source, b.Source)
}
func sameSurface(a, b surfacegeom.Surface) bool {
	switch x := a.(type) {
	case nil:
		return b == nil
	case surfacegeom.Plane:
		y, ok := b.(surfacegeom.Plane)
		return ok && sameFrame(x.Frame, y.Frame)
	case surfacegeom.Cylinder:
		y, ok := b.(surfacegeom.Cylinder)
		return ok && sameVec(x.Origin, y.Origin) && sameVec(x.Axis, y.Axis) && sameValue(x.Radius, y.Radius)
	case surfacegeom.Cone:
		y, ok := b.(surfacegeom.Cone)
		return ok && sameVec(x.Origin, y.Origin) && sameVec(x.Axis, y.Axis) && sameValue(x.Radius, y.Radius) && sameValue(x.HalfAngle, y.HalfAngle)
	case surfacegeom.Sphere:
		y, ok := b.(surfacegeom.Sphere)
		return ok && sameVec(x.Center, y.Center) && sameValue(x.Radius, y.Radius)
	case surfacegeom.Torus:
		y, ok := b.(surfacegeom.Torus)
		return ok && sameVec(x.Center, y.Center) && sameVec(x.Axis, y.Axis) && sameValue(x.Major, y.Major) && sameValue(x.Minor, y.Minor)
	case surfacegeom.Faceted:
		y, ok := b.(surfacegeom.Faceted)
		return ok && sameValue(x.Bound, y.Bound)
	case surfacegeom.NURBSSurface:
		_, ok := b.(surfacegeom.NURBSSurface)
		return ok
	default:
		return false
	}
}
func sameRat(a, b *big.Rat) bool {
	if a == nil || b == nil {
		return a == nil && b == nil
	}
	return a.Num().BitLen() <= 4096 && a.Denom().BitLen() <= 4096 && b.Num().BitLen() <= 4096 && b.Denom().BitLen() <= 4096 && a.Num().Cmp(b.Num()) == 0 && a.Denom().Cmp(b.Denom()) == 0
}
func sameInterval(a, b proofbound.RatInterval) bool {
	return sameRat(a.Lo, b.Lo) && sameRat(a.Hi, b.Hi)
}
func sameDenoted(a, b *surfacenormal.Revolved) bool {
	if a == nil || b == nil {
		return a == nil && b == nil
	}
	if a.Valid != b.Valid || a.Circular != b.Circular || a.Cap != b.Cap || (a.Runs == nil) != (b.Runs == nil) || (a.Ends == nil) != (b.Ends == nil) || len(a.Runs) != len(b.Runs) || len(a.Ends) != len(b.Ends) {
		return false
	}
	for i := range a.Origin {
		if !sameInterval(a.Origin[i], b.Origin[i]) {
			return false
		}
	}
	for i := range a.Basis {
		for j := range a.Basis[i] {
			if !sameInterval(a.Basis[i][j], b.Basis[i][j]) {
				return false
			}
		}
	}
	for i := range a.Runs {
		for j := range a.Runs[i] {
			if !sameInterval(a.Runs[i][j], b.Runs[i][j]) {
				return false
			}
		}
	}
	for i := range a.Ends {
		for j := range a.Ends[i] {
			for k := range a.Ends[i][j] {
				if !sameInterval(a.Ends[i][j][k], b.Ends[i][j][k]) {
					return false
				}
			}
		}
	}
	for i := range a.Centre {
		if !sameInterval(a.Centre[i], b.Centre[i]) {
			return false
		}
	}
	for i := range a.CapDir {
		if !sameInterval(a.CapDir[i], b.CapDir[i]) {
			return false
		}
	}
	return sameInterval(a.Radius, b.Radius)
}
func sameShiftGroup(a, b facetedrecord.Group) bool {
	return (a.Origins == nil) == (b.Origins == nil) &&
		(a.Upstream == nil) == (b.Upstream == nil) &&
		slices.Equal(a.Origins, b.Origins) &&
		slices.Equal(a.Upstream, b.Upstream) &&
		a.Source == b.Source &&
		sameVec(a.Centroid.Value, b.Centroid.Value) &&
		a.Centroid.Exactness == b.Centroid.Exactness &&
		sameValue(a.Centroid.Bound, b.Centroid.Bound) &&
		a.HasCentroid == b.HasCentroid &&
		sameSurface(a.Surface, b.Surface) &&
		a.Retained == b.Retained &&
		sameFloat(a.AxialDelta, b.AxialDelta) &&
		a.HasAxialDelta == b.HasAxialDelta &&
		sameDenoted(a.Denoted, b.Denoted) &&
		sameFloat(a.NormalBound, b.NormalBound) &&
		a.Reversed == b.Reversed &&
		a.Planar == b.Planar
}
