package loftcylinder

import (
	"context"
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/loftmesh"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/orderedwork"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// ChamferReachInput is everything the reach proofs read: the source payload
// FP, the finished mesh, the source records and the published allowance.
// Bands holds each cap's band triangle range and Middle the retained middle
// side's range in Mesh.Tris.
type ChamferReachInput struct {
	FP             facetedrecord.Payload
	Mesh           *facetedrecord.JoinedMesh
	Source         ChamferSource
	Ideal          []proofarith.Xpt
	Bands          [2][2]int
	Middle         [2]int
	ClipBound      float64
	Allowance      float64
	MultipleRings  bool
	ReflectedShift []*big.Rat
}

func (in *ChamferReachInput) nearBand(ctx context.Context) error {
	return chamferNearBandReach(ctx, in.Mesh, in.FP, in.Source, in.Ideal, in.Bands,
		in.ClipBound, in.Allowance, in.MultipleRings, in.ReflectedShift)
}

// ChamferReach runs the near-band proof, the bore inheritance
// check and the middle-side proof, and returns the error a serial run of the
// three in that order returns. With workers < 2 it is that serial run.
// Otherwise the near band and the middle facets share one flat fan-out: each
// facet runs on a forked grid, and the visit log of each is replayed on the
// real grid in facet order. From the first facet whose replay the real grid
// would refuse, the facets run serially.
func ChamferReach(ctx context.Context, in *ChamferReachInput, workers int) error {
	if workers < 2 {
		if err := in.nearBand(ctx); err != nil {
			return err
		}
		if err := chamferBoreInheritance(in.FP, in.Mesh, in.Source); err != nil {
			return err
		}
		return chamferMiddleSideReach(ctx, in.FP, in.Mesh, in.Source, in.Ideal,
			in.Middle, in.ClipBound)
	}
	inheritErr := chamferBoreInheritance(in.FP, in.Mesh, in.Source)
	var prep *chamferMiddleSidePrep
	var prepErr error
	if inheritErr == nil {
		prep, prepErr = newChamferMiddleSidePrep(in.FP, in.Mesh, in.Source, in.Ideal, in.Middle, in.ClipBound)
	}
	if inheritErr != nil || prepErr != nil {
		if err := in.nearBand(ctx); err != nil {
			return err
		}
		if inheritErr != nil {
			return inheritErr
		}
		return prepErr
	}
	facets := in.Middle[1] - in.Middle[0]
	// Each item has its own context derived from the caller's values only;
	// the polling caller alone cancels them.
	base := context.WithoutCancel(ctx)
	privates := make([]context.Context, 1+facets)
	cancels := make([]context.CancelFunc, 1+facets)
	for k := range privates {
		privates[k], cancels[k] = context.WithCancel(base) //nolint:fatcontext
	}
	cancelAll := func() {
		for _, cancel := range cancels {
			cancel()
		}
	}
	defer cancelAll()
	states := make(chan *ChamferMiddleState, workers)
	for range workers {
		states <- NewChamferMiddleState(prep.grid.Fork(),
			meshbool.NewParityMesh(in.FP.Verts, in.FP.Tris))
	}
	logs := make([][]loftmesh.ChamferGridQuery, facets)
	poll := func() error {
		if err := ctx.Err(); err != nil {
			cancelAll()
			return err
		}
		return nil
	}
	first, err := orderedwork.Each(1+facets, workers, 1, poll, func(k int) error { //nolint:contextcheck // private contexts, see above
		if k == 0 {
			return in.nearBand(privates[0])
		}
		st := <-states
		defer func() { states <- st }()
		err := ChamferMiddleFacet(privates[k], prep.proof, in.Middle[0]+k-1, st)
		logs[k-1] = st.Grid.TakeLog()
		return err
	})
	if first <= 0 {
		// A poll failure, or the near-band error, which a serial run reaches
		// before any facet.
		return err
	}
	// Facets 0..last-1 (the failing one included) are the ones the serial run
	// reaches; replay them on the real grid.
	last := facets
	if first < 1+facets {
		last = first
	}
	for t := range last {
		if !prep.grid.Replay(logs[t]) {
			return prep.serialFrom(ctx, in.Middle[0]+t)
		}
	}
	return err
}

// Cut's authenticated bore wall is kept verbatim. Its same-height radial
// source correspondence remains valid because the new outer cap bands stay
// clear of the entire source bore shell and both cap annuli keep their inner
// loops. The caller audits the rebuilt annuli before publishing the result.
func chamferBoreInheritance(fp facetedrecord.Payload, mesh *facetedrecord.JoinedMesh,
	source ChamferSource) error {
	if source.BoreGroup < 0 {
		if fp.LoftCylinderBore != nil || len(source.Inner[0]) != 0 || len(source.Inner[1]) != 0 {
			return fmt.Errorf("%w: chamfer has inconsistent no-bore cap records", decaderr.ErrUnsupported)
		}
		return nil
	}
	record := fp.LoftCylinderBore
	if record == nil || source.BoreGroup != record.WallGroup ||
		source.BoreRadius != record.BoreRadius || source.BoreBound != record.BoreBound ||
		source.BoreBound < 0 || proofbound.IsNonFinite(source.BoreBound) ||
		source.BoreBound > fp.MeshBound ||
		proofbound.AbsSumUpper(source.BoreRadius, source.BoreBound, source.InnerInset) >= source.BlankRadius {
		return fmt.Errorf("%w: chamfer bore lost its source radial bound", decaderr.ErrUnsupported)
	}
	if len(mesh.Verts) < len(fp.Verts) || len(fp.Tris) != len(fp.Src) ||
		len(mesh.Tris) != len(mesh.Src) {
		return fmt.Errorf("%w: chamfer bore has incomplete held triangles", decaderr.ErrUnsupported)
	}
	for cap, ring := range source.Inner {
		if len(ring) < 3 || len(ring) != len(record.Inner[cap]) {
			return fmt.Errorf("%w: chamfer bore lost its inner cap loop", decaderr.ErrUnsupported)
		}
		for i, index := range ring {
			if index != record.Inner[cap][i] || index < 0 || index >= len(fp.Verts) ||
				mesh.Verts[index] != fp.Verts[index] {
				return fmt.Errorf("%w: chamfer bore changed an inner cap station", decaderr.ErrUnsupported)
			}
		}
	}
	old := make([][3]int, 0)
	for i, group := range fp.Src {
		if group == source.BoreGroup {
			old = append(old, fp.Tris[i])
		}
	}
	if len(old) == 0 {
		return fmt.Errorf("%w: chamfer bore has no retained wall", decaderr.ErrUnsupported)
	}
	kept := 0
	for i, group := range mesh.Src {
		if group != source.BoreGroup {
			continue
		}
		if kept >= len(old) || mesh.Tris[i] != old[kept] {
			return fmt.Errorf("%w: chamfer changed a held bore-wall facet", decaderr.ErrUnsupported)
		}
		for _, index := range mesh.Tris[i] {
			if index < 0 || index >= len(fp.Verts) || mesh.Verts[index] != fp.Verts[index] {
				return fmt.Errorf("%w: chamfer changed a held bore-wall vertex", decaderr.ErrUnsupported)
			}
		}
		kept++
	}
	if kept != len(old) {
		return fmt.Errorf("%w: chamfer dropped a held bore-wall facet", decaderr.ErrUnsupported)
	}
	return nil
}
