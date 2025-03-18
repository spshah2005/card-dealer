#ifndef PLAYER_H
#define PLAYER_H

#include "card.h"

const int MAX_HAND_SIZE = 3;

typedef struct Player {
  
  Card hand[MAX_HAND_SIZE];
  int num_cards;

} // Player{}

// Returns a new player with no cards in their hand
Player Player_init() {
  Player temp;
  temp->num_cards = 0;
  return temp;
} // Player_init()

// Gives the player the card (WHOA!) (if they have room in their hand)
void Player_dealCard(Player* player, Card card) {
  // If player's hand is full, do nothing
  if (player->num_cards >= MAX_HAND_SIZE) return;
  player->hand[player->num_cards] = card;
} // Player_dealCard()

// Returns 1 if the player has a flush, 0 otherwise
int Player_hasFlush(Player* player) {
  // 0 or 1 cards counts as a flush
  if (player->num_cards < 2) return 1;
  // Compare every card to 1st card, if any differ it's not a flush
  uint8_t suit = Card_getSuit(player->hand[0]);
  for (int i = 0; i < player->num_cards; i++) {
    if (Card_getSuit(player->hand[i]) != suit) {
      return 0;
    }
  } // for
  return 1;
} // Player_hasFlush()

#endif
