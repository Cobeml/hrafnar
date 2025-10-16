#!/usr/bin/env python3
"""
Image Similarity Matcher using CLIP Embeddings
Enables visual person re-identification for memory-based recognition
"""

import torch
import numpy as np
from PIL import Image
from transformers import CLIPProcessor, CLIPModel
from typing import List, Tuple, Optional
from sklearn.metrics.pairwise import cosine_similarity


class ImageSimilarityMatcher:
    """
    Uses CLIP (Contrastive Language-Image Pre-training) to generate
    image embeddings for person re-identification.

    Features:
    - Generate 512-dimensional embeddings from PIL images
    - Compare embeddings using cosine similarity
    - Find best matches from a database of stored embeddings
    """

    def __init__(self, model_name: str = "openai/clip-vit-base-patch32", device: str = "cuda"):
        """
        Initialize CLIP model for image embedding generation.

        Args:
            model_name: Hugging Face model identifier (default: CLIP ViT-B/32)
            device: Device to run inference on ('cuda' or 'cpu')
        """
        print(f"Loading CLIP model: {model_name}...")
        self.device = device if torch.cuda.is_available() else "cpu"

        self.model = CLIPModel.from_pretrained(model_name).to(self.device)
        self.processor = CLIPProcessor.from_pretrained(model_name)

        self.model.eval()  # Set to evaluation mode
        print(f"✅ CLIP model loaded on {self.device}")

    def get_embedding(self, pil_image: Image.Image) -> np.ndarray:
        """
        Generate CLIP embedding for a PIL image.

        Args:
            pil_image: PIL Image object (RGB)

        Returns:
            NumPy array of shape (512,) containing the image embedding
        """
        # Preprocess image
        inputs = self.processor(images=pil_image, return_tensors="pt")
        inputs = {k: v.to(self.device) for k, v in inputs.items()}

        # Generate embedding
        with torch.no_grad():
            image_features = self.model.get_image_features(**inputs)

        # Normalize embedding (CLIP recommendation)
        embedding = image_features / image_features.norm(dim=-1, keepdim=True)

        # Convert to NumPy array
        embedding_np = embedding.cpu().numpy().flatten()

        return embedding_np

    def compare_embeddings(self, emb1: np.ndarray, emb2: np.ndarray) -> float:
        """
        Compute cosine similarity between two embeddings.

        Args:
            emb1: First embedding (shape: (512,))
            emb2: Second embedding (shape: (512,))

        Returns:
            Similarity score in range [0, 1] where 1 = identical
        """
        # Reshape for sklearn
        emb1_2d = emb1.reshape(1, -1)
        emb2_2d = emb2.reshape(1, -1)

        # Compute cosine similarity
        similarity = cosine_similarity(emb1_2d, emb2_2d)[0][0]

        # Convert from [-1, 1] to [0, 1] range (CLIP embeddings are normalized)
        # In practice, CLIP similarities are already in [0, 1] due to normalization
        similarity = float(similarity)

        return similarity

    def find_best_match(
        self,
        query_embedding: np.ndarray,
        stored_embeddings: List[Tuple[int, np.ndarray]],
        threshold: float = 0.85
    ) -> Optional[Tuple[int, float]]:
        """
        Find the best matching stored embedding for a query embedding.

        Args:
            query_embedding: Embedding to match (shape: (512,))
            stored_embeddings: List of (person_id, embedding) tuples
            threshold: Minimum similarity score to consider a match (default: 0.85)

        Returns:
            Tuple of (person_id, similarity_score) if match found above threshold
            None if no match found
        """
        if not stored_embeddings:
            return None

        best_match_id = None
        best_similarity = 0.0

        for person_id, stored_emb in stored_embeddings:
            similarity = self.compare_embeddings(query_embedding, stored_emb)

            if similarity > best_similarity:
                best_similarity = similarity
                best_match_id = person_id

        # Only return match if above threshold
        if best_similarity >= threshold:
            return (best_match_id, best_similarity)
        else:
            return None

    def batch_compare(
        self,
        query_embedding: np.ndarray,
        stored_embeddings: List[np.ndarray]
    ) -> np.ndarray:
        """
        Efficiently compare one query embedding against many stored embeddings.

        Args:
            query_embedding: Single embedding to compare (shape: (512,))
            stored_embeddings: List of embeddings to compare against

        Returns:
            NumPy array of similarity scores (shape: (len(stored_embeddings),))
        """
        if not stored_embeddings:
            return np.array([])

        # Stack embeddings into matrix
        stored_matrix = np.vstack(stored_embeddings)  # Shape: (N, 512)
        query_2d = query_embedding.reshape(1, -1)     # Shape: (1, 512)

        # Compute all similarities at once
        similarities = cosine_similarity(query_2d, stored_matrix)[0]

        return similarities


def test_image_similarity():
    """Test function to verify image similarity matching works"""
    print("\n" + "="*60)
    print("Testing Image Similarity Matcher")
    print("="*60)

    matcher = ImageSimilarityMatcher()

    # Create test images (solid colors as simple test)
    img1 = Image.new('RGB', (224, 224), color='red')
    img2 = Image.new('RGB', (224, 224), color='red')    # Same color
    img3 = Image.new('RGB', (224, 224), color='blue')   # Different color

    print("\nGenerating embeddings...")
    emb1 = matcher.get_embedding(img1)
    emb2 = matcher.get_embedding(img2)
    emb3 = matcher.get_embedding(img3)

    print(f"Embedding shape: {emb1.shape}")
    print(f"Embedding dtype: {emb1.dtype}")

    print("\nComparing embeddings...")
    sim_same = matcher.compare_embeddings(emb1, emb2)
    sim_diff = matcher.compare_embeddings(emb1, emb3)

    print(f"Similarity (red vs red): {sim_same:.4f}")
    print(f"Similarity (red vs blue): {sim_diff:.4f}")

    # Test find_best_match
    stored = [(1, emb2), (2, emb3)]
    match = matcher.find_best_match(emb1, stored, threshold=0.85)

    if match:
        print(f"\nBest match: Person ID {match[0]} with similarity {match[1]:.4f}")
    else:
        print("\nNo match found above threshold")

    print("\n✅ Test complete!")


if __name__ == "__main__":
    test_image_similarity()
