package capblend_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/capblend"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/offset2d"
	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/stretchr/testify/require"
)

func lineWalk() survey2d.SideWalk {
	return survey2d.SideWalk{Kind: survey2d.WalkLine}
}

func circleWalk() survey2d.SideWalk {
	return survey2d.SideWalk{Kind: survey2d.WalkCircular}
}

// Every gate of the loft cap-band volume proof refuses before any mesh, motion
// or slab is read; the proof itself runs in the root package's loft cap-band
// tests on a real tessellation.
func TestProveLoftBandVolumeGates(t *testing.T) {
	t.Parallel()

	both := capblend.Payload{StartLoops: map[int]bool{0: true}, EndLoops: map[int]bool{0: true}}
	outer := func() capblend.LoopMesh {
		return capblend.LoopMesh{Walks: []survey2d.SideWalk{lineWalk()}, ArcCount: []int{0},
			Joins: []offset2d.Join{{}}}
	}
	bore := capblend.LoopMesh{Whole: true, Walks: []survey2d.SideWalk{circleWalk()}}
	refused := func(name, msg string, cbp capblend.Payload, lms ...capblend.LoopMesh) {
		err := capblend.ProveLoftBandVolume(t.Context(), &topo.Mesh{}, &topo.Body{}, cbp, lms, nil)
		require.ErrorIs(t, err, decaderr.ErrUnsupported, name)
		require.ErrorContains(t, err, msg, name)
	}

	const bands = "requires both complete outer bands and at most one bore"
	refused("no loops", bands, both)
	refused("three loops", bands, both, outer(), bore, bore)
	refused("outer not chamfered at the start", bands,
		capblend.Payload{StartLoops: map[int]bool{0: false}, EndLoops: map[int]bool{0: true}}, outer())
	refused("outer not chamfered at the end", bands,
		capblend.Payload{StartLoops: map[int]bool{0: true}, EndLoops: map[int]bool{0: false}}, outer())
	refused("extra start loop", bands,
		capblend.Payload{StartLoops: map[int]bool{0: true, 1: true}, EndLoops: map[int]bool{0: true}}, outer())
	refused("extra end loop", bands,
		capblend.Payload{StartLoops: map[int]bool{0: true}, EndLoops: map[int]bool{0: true, 1: true}}, outer())

	const circular = "requires an unchanged circular bore"
	notWhole := bore
	notWhole.Whole = false
	refused("bore not whole", circular, both, outer(), notWhole)
	twoWalks := bore
	twoWalks.Walks = []survey2d.SideWalk{circleWalk(), circleWalk()}
	refused("bore with two walks", circular, both, outer(), twoWalks)
	straightBore := bore
	straightBore.Walks = []survey2d.SideWalk{lineWalk()}
	refused("straight bore", circular, both, outer(), straightBore)

	curved := outer()
	curved.Walks = []survey2d.SideWalk{lineWalk(), circleWalk()}
	refused("curved outer wall", "requires straight held outer walls", both, curved)

	reflex := outer()
	reflex.Joins = []offset2d.Join{{Arc: true}}
	refused("reflex corner without a fan", "reflex corner has no chorded apex fan", both, reflex)
}
