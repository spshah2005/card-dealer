// Card struct
#ifndef CARD_H
#define CARD_H

#include "stdlib.h"

#include "rfid.h"

// TODO: LUT of UID values to card values
uint8_t UID_to_value[52];

// DEBUG: suit enum
typedef enum {
  CLUBS,
  DIAMONDS,
  HEARTS,
  SPADES
} Suit; // Suit{}

// DEBUG: rank enum
typedef enum {
  TWO,
  THREE,
  FOUR,
  FIVE,
  SIX,
  SEVEN,
  EIGHT,
  NINE,
  TEN,
  JACK,
  QUEEN,
  KING,
  ACE        // In 3 card poker Ace is high except for making straights, so I think this ordering makes sense
} Rank; // Rank{}

const uint16_t NUM_SUITS = 4; // I don't think these matter
const uint16_t NUM_RANKS = 13;

typedef struct Card {
  /* Bits 7-6: unused
  *  Bits 5-4: suit (0b00 Clubs, 0b01 Diamonds, 0b10 Hearts, 0b11 Spades)
  *  Bits 3-0: rank (0x0 Two, 0x1 Three, ... 0xB King, 0xC Ace)
  *  e.g. 0x2C = Ace of Hearts, 0x31 = Two of Spades
  */
  uint8_t value;
}; // Card{}

Card Card_init(Rank rank, Suit suit) {
  Card temp;
  temp.value = (rank) | (suit << 4);
  return temp;
} // Card_rank_and_suit()

Card Card_initFromValue(uint8_t value) {
  Card temp = {value};
  return temp;
} // Card_init()

// Returns the rank of the card as an unsigned int 0-12
inline uint8_t Card_rank(const Card& card) {
  return ((card.value) & 0xF);
} // Card_rank()

// Returns the difference between the rank of card 1 and card 2 (needed because Aces can be low)
int Card_rankDiff(const Card& card1, const Card& card2) {
  uint8_t rank1 = Card_rank(card1);
  uint8_t rank2 = Card_rank(card2);

  // 1st card is Ace, 2nd card is 2
  if (rank1 == 12 && rank2 == 0) return -1;
  // 1st card is 2, 2nd card is Ace
  if (rank1 == 0 && rank2 == 12) return 1;

  return rank1 - rank2;
} // Card_rank_diff()

// Returns the card's suit for comparison (0x10 clubs, 0x20 diamonds, 0x30 hearts, 0x40 spades)
inline uint8_t Card_suit(const Card& card) {
  return ((card.value) & 0xF0);
} // Card_suit()

// Returns 1 if cards are of the same suit, 0 otherwise
int Card_sameSuit(const Card& card1, const Card& card2) {
  return (Card_suit(card1) == Card_suit(card2));
} // Card_suit_comp()

#endif
