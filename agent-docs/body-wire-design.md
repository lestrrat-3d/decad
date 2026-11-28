# Read-only body wire format

The analytic and held-facet STEP frames are implemented. General body geometry is not yet
supported.

## Purpose

Send body geometry to a read-only receiver that chooses its own tessellation tolerance.
The format uses STEP's surface and topology vocabulary; it does not serialize a `decad.Body`
or make the received geometry a modeling operand. Rendering is one consumer.

## Binary frame

Write ASCII `DCS1` for analytic STEP, `DCS2` for faceted STEP, or `DCS3` for sheet facets;
then a uint32 little-endian compressed length, a uint32 uncompressed length, and a zero
uint32. Follow the header with one gzip-compressed payload. `DCS1` and `DCS2` carry STEP
Part 21; `DCS3` carries the sheet facet record below.
Each length is at most 64 MiB. `Read` consumes one frame and leaves subsequent frames unread.
Reject malformed headers, unsupported versions, extra gzip members, size mismatches and
decompression past the declared size. A write error may leave a partial frame in the stream.

Use a fixed STEP header and gzip metadata, so equal bodies and equal sender verification
tolerances give equal bytes. A sender supplies a positive length for STEP's boundary check;
this does not set the receiver's tessellation tolerance.

## Current producer and receiver

`bodywire.Write` calls `export.NewSTEPFile`, preferring its analytic product. For a source
that holds a `facetedrecord.Payload`, it calls `export.NewFacetedSTEPFile` to preserve its
held facets and their bound. If an analytic source cannot be exported analytically, accept
the faceted fallback and report its source boundary bound. Parse the written STEP and pass
it through the actual receiver's tessellator before writing a frame. If the receiver does not
support analytic output, retry with faceted STEP. Refuse if the receiver cannot read the
fallback. The public STEP product description distinguishes the writer's analytic and faceted paths (`agent-docs/step-export-design.md`).

A faceted STEP product uses `DCS2` in place of `DCS1` and appends an IEEE binary64 source
boundary bound in millimetres to the 16-byte header, before the gzip bytes. Its bound is the
source `Mesh.Bound()` at the sender's verification tolerance. Its STEP faces are that mesh's
own triangles (`agent-docs/step-export-design.md` Geometry contract), and the STEP reader
triangulates each three-vertex face using its original coordinates. For an analytic source
this is a sender-side approximation. The producer only sends `DCS2` after that real receiver
accepts its STEP. The frame records the source's bound as the sender's claim; it does not
authenticate it or establish the source's proofs at the receiver.

`Body.SourceBoundaryBound` returns the sender's claimed bound and true for `DCS2` or `DCS3`;
for `DCS1` it returns false. A `DCS2` tessellation request below the stored bound is refused,
because the held facets cannot be refined to meet it. The root adapter in `reader_install.go`
reads the private `Body.n.Payload` solely to identify `facetedrecord.Payload`, then passes
that decision to `internal/stepwire`; it does not expose a root handle to internal algorithms.

## Sheet facet record

The wire writer sends sheets as their boundary-verified tessellation in a
`DCS3` frame. It has the same 24-byte header and source-bound field as `DCS2`. Its gzip payload
starts with uint64 vertex count and uint64 triangle count. The vertices follow as IEEE binary64
little-endian `(x,y,z)` triples in millimetres; triangles follow as uint32 little-endian index
triples. Copy the mesh's coordinates, indices and positive-side winding unchanged. The record
does not claim that the sheet encloses volume. Refuse nonfinite values, empty meshes, invalid
indices, counts beyond the frame and triangle limits, and arithmetic overflow before allocation.
The receiver returns the same facets at tolerances no smaller than the sender's bound; it does
not claim to refine them. `Body.IsSheet` distinguishes this record from the solid STEP records.

`bodywire.Read` retains the parsed STEP model. `Body.Tessellate` reads that model at the
receiver's positive length and returns a `MeshBody`: its geometry and measurements describe
the mesh itself. It does not claim a proven boundary bound to the original decad body. The
reader handles the STEP geometry in `agent-docs/format-io-design.md` STEP import, including
planes, cylinders, cones and the admitted spherical cap. Curved walls and circular edges are
sampled only at tessellation time. Unsupported STEP entities return `ErrUnsupported`.

`internal/stepbrep.ReadAtTolerance` accepts the requested circular chord. Its edge sampler
refuses a sample count above the triangle limit before allocation. `ReadSTEP` retains its
existing 0.01 mm chord. The sender's verification tolerance must not become the receiver's
requested chord or a claim about the decoded geometry.

The current STEP writer first performs a boundary-verified tessellation and handles solids and sheets.
It emits analytic surfaces only for the cases `agent-docs/step-export-design.md` admits.
Neither that admission nor the STEP reader covers all decad bodies. Sheets use the facet
fallback above. Other held-facet payload classes use the STEP faceted fallback when admitted.

## Required expansion

- Preserve each supported body's held geometry, topology, orientation and applicable error
  bounds. An approximate stand-in must not be described as exact.
- Extend exact held-facet transfer beyond `facetedrecord.Payload`. Keep a sender-side
  tessellation fallback distinguishable from analytic geometry and carry its bound.
- Extend analytic support to sheets, multiple shells and lumps, bounded analytic stand-ins,
  and free-form surfaces and curves. Refuse unsupported classes explicitly until their records
  and receiver behavior are implemented.
- Define resource limits and cancellation through parsing, sampling and triangulation.
  Reject malformed references and oversized data before count-based allocation.
- Prove each geometry class with a real decad producer, the real frame encoder and decoder,
  and receiver-side tessellation. Compare computed geometry at more than one tolerance for
  curved bodies. The receiver must not claim decad's proofs without establishing them.

## Rotation note

The previous custom-record prototype refused general rotations because `r3.FromBasis`
orthonormalizes a stored basis. STEP carries the transformed geometry instead; it avoids that
specific decoder path but does not itself establish an error bound to the original body.
Keep the distinction until STEP transport has an explicit bound for that conversion.

The STEP producer writes a rotated full cylinder as an analytic surface with two circular
bounds when its source seam points do not align exactly. The wire tests cover a rotated solid
cylinder and a rotated plate with a circular hole. `agent-docs/step-export-design.md` owns the
writer's construction and its existing precision limitations.
