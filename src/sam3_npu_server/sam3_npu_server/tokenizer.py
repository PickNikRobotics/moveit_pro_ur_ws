# Copyright 2026 PickNik Inc.
# SPDX-License-Identifier: BSD-3-Clause
"""SAM3's text tokenizer (OpenAI CLIP's byte-level BPE), with no torch, ftfy or regex dependency.

A port of the tokenizer in Meta's sam3 package (sam3/model/tokenizer_ve.py, itself from open_clip and
OpenAI CLIP) that returns the same ids for every prompt in test/data/sam3_reference_tokens.json: start
token, BPE ids, end token, zero padding to the context length, and truncation that keeps the end token.
Two simplifications, neither of which changes an ASCII prompt: ftfy.fix_text (mojibake repair) is not
applied, and the `regex` classes \\p{L} and \\p{N} are written with the standard `re` module's Unicode
classes. The vocabulary is CLIP's bpe_simple_vocab_16e6.txt.gz (MIT, see assets/CLIP_LICENSE).
"""

import functools
import gzip
import html
import os
import re

VOCAB_FILE = "bpe_simple_vocab_16e6.txt.gz"


def default_vocab_path() -> str:
    """The vocabulary installed in the package's share/assets, or beside the source when not installed (tests)."""
    try:
        from ament_index_python.packages import get_package_share_directory

        installed = os.path.join(get_package_share_directory("sam3_npu_server"), "assets", VOCAB_FILE)
        if os.path.isfile(installed):
            return installed
    except Exception:  # noqa: BLE001 -- not installed, or no ament index: use the source tree
        pass
    return os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "assets", VOCAB_FILE)


@functools.lru_cache()
def bytes_to_unicode():
    """CLIP's reversible byte -> printable-unicode map, so BPE never sees whitespace or control bytes."""
    bs = list(range(ord("!"), ord("~") + 1)) + list(range(ord("¡"), ord("¬") + 1)) + list(range(ord("®"), ord("ÿ") + 1))
    cs = bs[:]
    n = 0
    for b in range(256):
        if b not in bs:
            bs.append(b)
            cs.append(256 + n)
            n += 1
    return dict(zip(bs, (chr(c) for c in cs)))


def _pairs(word):
    return {(a, b) for a, b in zip(word, word[1:])}


def _clean(text: str) -> str:
    """sam3's "lower" clean: html-unescape twice, collapse whitespace, lowercase."""
    text = html.unescape(html.unescape(text)).strip()
    return re.sub(r"\s+", " ", text).strip().lower()


class Sam3Tokenizer:
    START = "<start_of_text>"
    END = "<end_of_text>"

    def __init__(self, vocab_path: str = "", context_length: int = 32):
        with gzip.open(vocab_path or default_vocab_path()) as f:
            merges = f.read().decode("utf-8").split("\n")
        merges = [tuple(m.split()) for m in merges[1:49152 - 256 - 2 + 1]]
        vocab = list(bytes_to_unicode().values())
        vocab = vocab + [v + "</w>" for v in vocab] + ["".join(m) for m in merges] + [self.START, self.END]
        self.encoder = {tok: i for i, tok in enumerate(vocab)}
        self.bpe_ranks = {m: i for i, m in enumerate(merges)}
        self.cache = {self.START: self.START, self.END: self.END}
        self.start_id = self.encoder[self.START]
        self.end_id = self.encoder[self.END]
        self.context_length = context_length
        # \p{L}+ is [^\W\d_]+ (word characters that are neither digits nor the underscore), \p{N} is \d,
        # and [^\s\p{L}\p{N}]+ is a run of anything else, the underscore included.
        self.pattern = re.compile(
            re.escape(self.START) + "|" + re.escape(self.END) +
            r"""|'s|'t|'re|'ve|'m|'ll|'d|[^\W\d_]+|\d|(?:[^\s\w]|_)+""", re.IGNORECASE)

    def _bpe(self, token: str) -> str:
        if token in self.cache:
            return self.cache[token]
        word = tuple(token[:-1]) + (token[-1] + "</w>",)
        pairs = _pairs(word)
        if not pairs:
            return token + "</w>"
        while True:
            bigram = min(pairs, key=lambda p: self.bpe_ranks.get(p, float("inf")))
            if bigram not in self.bpe_ranks:
                break
            first, second = bigram
            merged, i = [], 0
            while i < len(word):
                try:
                    j = word.index(first, i)
                except ValueError:
                    merged.extend(word[i:])
                    break
                merged.extend(word[i:j])
                i = j
                if word[i] == first and i < len(word) - 1 and word[i + 1] == second:
                    merged.append(first + second)
                    i += 2
                else:
                    merged.append(word[i])
                    i += 1
            word = tuple(merged)
            if len(word) == 1:
                break
            pairs = _pairs(word)
        out = " ".join(word)
        self.cache[token] = out
        return out

    def encode(self, text: str) -> list:
        """BPE ids of the cleaned text, without start or end tokens."""
        byte_encoder = bytes_to_unicode()
        ids = []
        for token in self.pattern.findall(_clean(text)):
            token = "".join(byte_encoder[b] for b in token.encode("utf-8"))
            ids.extend(self.encoder[t] for t in self._bpe(token).split(" "))
        return ids

    def __call__(self, text: str) -> list:
        """The model's token row: start, ids, end, zero-padded; truncated to keep the end token."""
        ids = [self.start_id] + self.encode(text) + [self.end_id]
        if len(ids) > self.context_length:
            ids = ids[: self.context_length]
            ids[-1] = self.end_id
        return ids + [0] * (self.context_length - len(ids))
