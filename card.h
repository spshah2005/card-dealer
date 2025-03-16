// Card struct
#ifndef CARD_H
#define CARD_H

#include "stdlib.h"

typedef struct Card {
  /* Bits 7-6: unused
  *  Bits 5-4: suit (0b00 Clubs, 0b01 Diamonds, 0b10 Hearts, 0b11 Spades)
  *  Bits 3-0: rank (0b0000 Two, 0b0001 Three, ... 0b1011 King, 0b1100 Ace)
  */
  uint8_t value;
}; // Card{}

// Returns the rank of the card as an unsigned int 0-12
inline uint8_t Card_rank(const Card& card) {
  return ((card.value) & 0xF);
} // Card_rank()

// Returns the difference between the rank of card 1 and card 2 (needed because Aces can be low)
int Card_rank_diff(const Card& card1, const Card& card2) {
  uint8_t rank1 = Card_rank(card1);
  uint8_t rank2 = Card_rank(card2);
  
  // 1st card is Ace, 2nd card is 2
  if (rank1 == 12 && rank2 == 0) return -1;
  // 1st card is 2, 2nd card is Ace
  if (rank1 == 0 && rank2 == 12) return 1;
  
  return rank1 - rank2;
} // Card_rank_diff()

inline uint8_t Card_suit(const Card& card) {
  return ((card.value) & 0xF0);
} // Card_suit()

// Returns 1 if cards are of the same suit, 0 otherwise
int Card_suits_comp(const Card& card1, const Card& card2) {
  return (Card_suit(card1) == Card_suit(card2));
} // Card_suit_comp()

#endif
