# Body digest

`Body.Digest` and `Document.PairDigest` give a body, or an ordered pair of bodies, a SHA-256 identity of everything
decad's evaluator reads from it. A caller uses the identity as a cache key: two pairs with equal pair digests give
the same `Intersect` answer, so one proof serves both. `internal/statedigest/` is the encoder, `body_digest.go` the
root adapter.

## 1. API

- `type Digest [32]byte`, with `String` giving 64 lowercase hex digits.
- `(*Body).Digest(ctx)` is the identity of one body. Equal digests answer the single-body queries that read only the body's
  own geometry alike (`Volume`, `Bounds`, `Tessellate`, `LocatePoint`, and the rest). `Origin()` returns the raw
  producer id, which the digest relabels, so it is not covered. The first call walks the body and polls `ctx`. Later calls
  return the stored digest.
- `(*Document).PairDigest(ctx, a, b)` is the identity of the ordered pair. `PairDigest(a, b)` and `PairDigest(b, a)`
  differ in general. Equal pair digests cover `Intersect` only, and only for two live bodies in one container.
- A nil body or nil `ctx` is `ErrDegenerate`. In `PairDigest`, `a == b` is `ErrDegenerate` and a body of another
  document is `ErrForeignBody`. A retired body has a digest like a live one. A value the encoder cannot state is
  `ErrUnsupported` naming its field path (§5).
- Both are queries (api-design §6.3). A digest is a decided identity, so it carries no `Exactness`.
- The digest hashes machine representations (float bits, Go type names). It holds within one build of the program
  and is not a file format.

## 2. What the walk covers

The walk starts at the root `*Body` handle and encodes every value reachable by pointer, slice, array, map,
interface and struct field, exported or not, except:

- the caches decad documents as result-neutral: `topo.Body.PlanarConvexity`, `PlanarSnapshot`, `PlanarHints`,
  `Tess` and `SweepRadii`, `Body.pairReports`, the digest's own memo `Body.digest`, `blindGroupProof.cache` and
  `occupancy.Record.grid` (an index of `Sites`, built from them by every constructor);
- back-pointers. A node struct's `Owner` field is never encoded, and a pointer to a node is encoded as its handle,
  so a node and its handle are one entry;
- every `*topo.Document` and `*Document`, which the document settings below replace.

For every body handle the walk reaches it also encodes the document records that name the handle: the producing
`Operation` (kind, inputs with role and use, the input handles, which the walk follows, outputs, face successors
and the identity seal), `circleBuilds`, `circlePlacements` and membership in `identityPlacements`. An ancestor named
by history is covered, and so is everything `historyProducers` hands the chain regroup. `consumers` is not read:
only the paired and blind Cut routes read it, and `Intersect` runs neither. The owning document's `gapReach`,
`chainTol`, `rimSplit` and the text of `gapErr` and `chainErr` come first in the hash.

Encoding rules:

- Every value is tagged by kind. Strings and slices are length-prefixed. Floats are their IEEE bits, so `-0` and
  `+0` differ, which only loses reuse. An interface carries its dynamic type's package path and name.
- A pointer or a non-empty slice is either a first visit (its target follows) or a back-reference to its visit
  order. Sharing between two parts of one body is part of the digest.
- A map is its length, then its entries ordered by the key's visit order (a pointer key the walk already met) or by
  an isolated encoding of the key. Two keys that encode alike are `ErrUnsupported`.
- A `provenance.Producer`, `denotation.LevelID` or `denotation.CurveID` of 1 or more is replaced by its first-visit
  order within its type. Zero and negative ids stay raw, because the evaluator reads them as "none". A body whose
  source is a document's first operation (producer 0) therefore never matches one whose source is not, which loses
  reuse and never soundness. The `int` fields that copy a producer (`digestIDFields`) are relabeled in the producer
  order, so each stays equal to the producer it copies.

`PairDigest` hashes a domain tag, both body digests and the shared structure: for each node, and each id, that both
walks met, the pair of its visit order in `a`'s walk and in `b`'s. A copy and its source, or two copies of one
source, share nodes that two identical independent bodies do not, and the evaluator reads that sharing by pointer
identity. Each body stores its walk's orders with its digest, so a pair costs one map intersection.

## 3. Collision argument

The encoding is injective on canonical forms: equal byte strings decode to the same types, scalars, sharing pattern
and id-equality pattern. Two different canonical forms share a digest only by a SHA-256 collision. For q digests
computed, the chance of any collision is at most q²/2^257. The inputs are the caller's own models.

## 4. Soundness: equal pair digests give equal answers

If `PairDigest(a, b) == PairDigest(a', b')`, then `Intersect(a, b)` and `Intersect(a', b')` return the same error
code, gap shift and volume bits. Equal canonical forms give a bijection between the values reachable in the two
pairs that keeps every type, scalar, pointer sharing, side-table entry, document setting and id equality, and maps
`a` to `a'` and `b` to `b'`. The claim then follows by induction over the evaluation's reads, given five conditions:

- **C1, coverage.** The evaluation reads only values of §2, plus `NextProducer` and the mint counters, and uses
  those only to name the uncommitted result.
- **C2, ids.** The evaluation compares ids only by equality and by sign.
- **C3, caches.** A skipped cache returns what recomputing would return.
- **C4, fresh ids.** The result's producer, levels and curves are fresh in both runs, and the outcome does not
  depend on their values.
- **C5, process.** Both runs use the same binary. Worker counts do not change results.

The message text of an error can differ where it prints an id. The claim covers codes, shifts and volumes.

| Condition | Held by |
|---|---|
| C1 | `TestDigestTypeCoverage` walks the static type graph from `Body`, following each non-empty interface into every implementer in the module (`topo.Payload` among them). It fails on a `sync` or `sync/atomic` value, function, channel or unsafe pointer that `digestSkipFields` does not skip and `digestRefuseTypes` or `digestRefuseFields` does not refuse. It also fixes the set of `interface{}` fields, whose dynamic values the encoder checks at run time. `TestDigestSideTables` classifies every `Document` field as read or unread with a reason. |
| C2 | `TestIdentityArithmeticAllowList` type-checks the module and lists every function that does more with an id than compare it for equality. Each is in an allow list with a reason: ids minted from the document counter, sign checks, text, and copies into the `digestIDFields` ints. A new one fails until it is read and listed. |
| C3 | Each skipped cache is documented result-neutral at its declaration, and `TestBodyDigestStableAfterCaches` requires the digest unchanged after `Tessellate`, `Verify`, `LocatePoint`, `ContactPair` and an empty `Intersect`. |
| C4 | `TestPairDigestRenumberedIDs` builds one construction after 1, 2 and 7 filler operations and requires equal digests and the same `Intersect` volume bits. A changed size and a different gap tolerance give different digests. |
| C5 | Not tested. It is a property of the caller's process. |

The pair relation is held by `TestPairDigestSharedSource` (two copies of one source against two copies of two
identical sources: equal body digests, different pair digests), and the reuse a cache key needs by
`TestBodyDigestMatchesMeshFingerprint` (equal digests exactly when the tessellations are bit-identical, over bodies
built at several history positions and sizes). `TestBodyDigestIncludesHistory` requires equal geometry with
different history to digest differently.

## 5. Refusals

The encoder refuses, with `ErrUnsupported` and the field path, what it cannot state:

- a non-nil function, channel or unsafe pointer, and a `sync` or `sync/atomic` value no field classification covers;
- two map keys that encode alike, and an isolated key encoding over 4 MiB;
- `sketch.Sketch` and `sketch.FitSpline`: the point-loft source and the cone certificates of a revolve-loft join keep
  live sketch entities, which their owner can still edit and which carry their own caches and functions;
- the lazily built pieces of an `occupancy.Leaf` (`pieces`, `inner`), which hold a closure over exact data the
  leaf's other fields do not state.

A refused body has no digest, and a caller that wants one proof per pair proves the pair itself.

## 6. Limits

- `PairDigest` covers `Intersect`-like evaluations. The paired and blind Cut routes read `consumers`, which the
  digest leaves out, so equal pair digests do not promise equal `Cut` answers.
- A global map in an internal package, keyed by a body, is not caught by the type tests. The evaluator has none.
