package decad

import (
	"context"
	"crypto/sha256"
	"encoding/binary"
	"encoding/hex"
	"errors"
	"fmt"
	"math"
	"reflect"

	"github.com/lestrrat-3d/decad/internal/denotation"
	"github.com/lestrrat-3d/decad/internal/occupancy"
	"github.com/lestrrat-3d/decad/internal/prooflineage"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/statedigest"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/sketch"
)

// Digest is a SHA-256 identity of the evaluator state of a body or a pair of
// bodies (agent-docs/body-digest-design.md).
type Digest [32]byte

// String returns the digest as 64 lowercase hex digits.
func (g Digest) String() string { return hex.EncodeToString(g[:]) }

// bodyDigestMemo is what Body.Digest stores: the digest and the walk that
// PairDigest reads the shared structure from.
type bodyDigestMemo struct {
	sum  Digest
	walk *statedigest.Walk
}

// Digest returns the identity of everything decad's evaluator can read from b:
// b and every value reachable from it, the document records that name a
// reached body, and the document's gap tolerance, chain tolerance and rim
// split. Producer, level and curve ids enter as the order in which the walk
// first meets them, so the digest does not depend on how many operations ran
// before. Containers, handle addresses and caches do not enter.
//
// Two bodies with equal digests answer the single-body queries that read
// only the body's own geometry identically (Volume, Bounds, Tessellate,
// LocatePoint, ...). Origin() is not among them: it returns the raw producer
// id, which the digest relabels. For a question about two bodies, use
// [Document.PairDigest]: two equal body digests do not cover what the two
// bodies share. The first call walks the body, polling ctx; later calls
// return the stored digest. A nil body or a nil ctx is [ErrDegenerate]. A
// value the encoding cannot state (a function, a channel, an unclassified
// synchronization field, two map keys that encode alike) is [ErrUnsupported]
// naming its field path.
//
// The digest is stable within one build of the program: it hashes machine
// representations, so it is not a file format.
func (b *Body) Digest(ctx context.Context) (Digest, error) {
	m, err := b.digestMemo(ctx)
	if err != nil {
		return Digest{}, err
	}
	return m.sum, nil
}

// PairDigest returns the identity of the pair (a, b) in that order: both body
// digests and every node and id the two walks share. Equal pair digests mean
// Intersect(a, b) gives the same answer
// (agent-docs/body-digest-design.md §4). They do not cover Cut or any other
// two-body evaluation: Cut reads operations and consumers, which the digest
// omits. Both bodies must be live and in one container; equal digests of
// bodies that break that precondition promise nothing. PairDigest(a, b) and
// PairDigest(b, a) differ in general. A body of another document is
// [ErrForeignBody]; a nil body, a == b or a nil ctx is [ErrDegenerate].
// Retired bodies are digested like live ones.
func (d *Document) PairDigest(ctx context.Context, a, b *Body) (Digest, error) {
	if a == nil || b == nil {
		return Digest{}, fmt.Errorf("%w: a nil body has no digest", ErrDegenerate)
	}
	if a == b {
		return Digest{}, fmt.Errorf("%w: a pair needs two distinct bodies", ErrDegenerate)
	}
	if a.n.Doc != documentNode(d) || b.n.Doc != documentNode(d) {
		return Digest{}, ErrForeignBody
	}
	ma, err := a.digestMemo(ctx)
	if err != nil {
		return Digest{}, err
	}
	mb, err := b.digestMemo(ctx)
	if err != nil {
		return Digest{}, err
	}
	h := sha256.New()
	h.Write([]byte("decad.PairDigest.v1"))
	h.Write(ma.sum[:])
	h.Write(mb.sum[:])
	h.Write(statedigest.Shared(ma.walk, mb.walk))
	var out Digest
	h.Sum(out[:0])
	return out, nil
}

func (b *Body) digestMemo(ctx context.Context) (*bodyDigestMemo, error) {
	if b == nil {
		return nil, fmt.Errorf("%w: a nil body has no digest", ErrDegenerate)
	}
	if ctx == nil {
		return nil, fmt.Errorf("%w: a nil context", ErrDegenerate)
	}
	if m := b.digest.Load(); m != nil {
		return m, nil
	}
	w, err := bodyDigestEncoder.Encode(ctx, documentSettingsKey(documentHandle(b.n.Doc)), reflect.ValueOf(b))
	if err != nil {
		if errors.Is(err, statedigest.ErrUnsupported) {
			return nil, fmt.Errorf("%w: %w", ErrUnsupported, err)
		}
		return nil, err
	}
	m := &bodyDigestMemo{sum: w.Sum, walk: w}
	if !b.digest.CompareAndSwap(nil, m) {
		return b.digest.Load(), nil
	}
	return m, nil
}

// documentSettingsKey encodes the document values the evaluator reads that no
// body holds: the gap tolerance, chain tolerance and rim split, and the text of
// the errors an invalid option named. d is nil for a body outside any document.
func documentSettingsKey(d *Document) []byte {
	out := []byte("decad.Digest.v1")
	if d == nil {
		return append(out, 0)
	}
	out = append(out, 1)
	out = binary.LittleEndian.AppendUint64(out, math.Float64bits(d.gapReach))
	out = appendErrText(out, d.gapErr)
	out = binary.LittleEndian.AppendUint64(out, math.Float64bits(d.chainTol))
	out = appendErrText(out, d.chainErr)
	if d.rimSplit {
		return append(out, 1)
	}
	return append(out, 0)
}

func appendErrText(out []byte, err error) []byte {
	if err == nil {
		return append(out, 0)
	}
	out = append(out, 1)
	out = binary.LittleEndian.AppendUint64(out, uint64(len(err.Error())))
	return append(out, err.Error()...)
}

// digestSkipFields lists the fields the digest leaves out. Each is a cache the
// evaluator documents as result-neutral, or the digest's own memo. Back-pointers
// (the Owner fields) are left out by the encoder's owner rule.
// TestDigestTypeCoverage fails on any other synchronization field in the graph.
var digestSkipFields = map[reflect.Type][]string{
	reflect.TypeFor[topo.Body](): {"PlanarConvexity", "PlanarSnapshot", "PlanarHints", "Tess", "SweepRadii"},
	reflect.TypeFor[Body]():      {"pairReports", "digest"},
	// cache holds a tessellation bound to the body and operation that built it.
	reflect.TypeFor[blindGroupProof](): {"cache"},
	// grid indexes Sites, and every constructor builds it from them.
	reflect.TypeFor[occupancy.Record](): {"grid"},
}

// digestRefuseTypes lists the types whose values Digest refuses with ErrUnsupported: live state the digest cannot
// capture. A sketch keeps solver state its owner can still edit, and every sketch entity, point loft source and cone
// certificate a payload holds links back to its sketch, so refusing the sketch refuses them all.
var digestRefuseTypes = map[reflect.Type]string{
	reflect.TypeFor[sketch.Sketch]():    "it is a live sketch its owner can still edit",
	reflect.TypeFor[sketch.FitSpline](): "it is a live sketch entity that caches its own evaluation",
}

// digestRefuseFields lists the fields Digest refuses while they are set. A lazily built piece holds a closure over
// exact data the other fields of its leaf do not state.
var digestRefuseFields = map[reflect.Type]map[string]string{
	reflect.TypeFor[occupancy.Leaf](): {
		"pieces": "an occupancy leaf builds its exact pieces lazily from a closure",
		"inner":  "an occupancy leaf builds its inner pieces lazily from a closure",
	},
}

// digestIDFields lists the integer fields that hold a producer copied out of an Origin: the contained-Cut lineage
// compares them with a body's producer. They are relabeled with the producer type so that equality holds.
var digestIDFields = func() map[reflect.Type]map[string]reflect.Type {
	producer := reflect.TypeFor[provenance.Producer]()
	return map[reflect.Type]map[string]reflect.Type{
		reflect.TypeFor[prooflineage.VolumeLineage]():       {"TargetProducer": producer, "ToolProducer": producer},
		reflect.TypeFor[prooflineage.SourceCentroid]():      {"Producer": producer},
		reflect.TypeFor[prooflineage.SourceMoments]():       {"Producer": producer},
		reflect.TypeFor[prooflineage.ContainedCutMoments](): {"resultProducer": producer},
	}
}()

var bodyDigestEncoder = statedigest.New(statedigest.Config{
	Skip:         digestSkipFields,
	IDFields:     digestIDFields,
	RefuseTypes:  digestRefuseTypes,
	RefuseFields: digestRefuseFields,
	Opaque:       []reflect.Type{reflect.TypeFor[*topo.Document](), reflect.TypeFor[*Document]()},
	IDs:          []reflect.Type{reflect.TypeFor[provenance.Producer](), reflect.TypeFor[denotation.LevelID](), reflect.TypeFor[denotation.CurveID]()},
	OwnerField:   "Owner",
	Extra:        bodyDigestSideTables,
})

var bodyPtrType = reflect.TypeFor[*Body]()

// bodyDigestSideTables returns the document records for a body handle: the
// operation that produced it, its circle build and placement witnesses and its
// identity-placement mark. They are what the history routes read through
// historyProducers and the circle replays. consumers is not read: only the
// paired and blind Cut routes read it. TestDigestSideTables lists every
// document table keyed by a body and says which this reads.
func bodyDigestSideTables(p reflect.Value) []reflect.Value {
	if p.Type() != bodyPtrType {
		return nil
	}
	h := (*Body)(p.UnsafePointer()) // p may come from an unexported field, so Interface would panic
	d := documentHandle(h.n.Doc)
	if d == nil {
		return nil
	}
	d.n.LiveMu.RLock()
	op := d.producers[h]
	cb := d.circleBuilds[h]
	cp := d.circlePlacements[h]
	_, idp := d.identityPlacements[h]
	d.n.LiveMu.RUnlock()
	return []reflect.Value{reflect.ValueOf(op), reflect.ValueOf(cb), reflect.ValueOf(cp), reflect.ValueOf(idp)}
}
