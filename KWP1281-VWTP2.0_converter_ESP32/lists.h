#ifndef LISTS_H
#define LISTS_H

// I apologize.

size_t get_list_count(uint8_t *list, size_t list_size)
{
  size_t list_count = list_size;
  for (size_t i = 0; i < list_count; i++)
  {
    if (list[i] == 0)
    {
      list_count = i;
      break;
    }
  }
  return list_count;
}

bool is_in_list(uint8_t *list, size_t list_size, uint8_t value)
{
  size_t list_count = get_list_count(list, list_size);
  for (size_t i = 0; i < list_count; i++)
  {
    if (list[i] == value)
    {
      return true;
    }
  }
  return false;
}

bool add_to_list(uint8_t *list, size_t list_size, uint8_t value)
{
  if (is_in_list(list, list_size, value))
  {
    return false;
  }
  
  size_t list_count = get_list_count(list, list_size);
  if (list_count == list_size)
  {
    return false;
  }

  for (uint8_t i = list_count; i > 0; i--)
  {
    list[i] = list[i - 1];
  }

  list[0] = value;
  return true;
}

size_t find_in_list(uint8_t *list, size_t list_size, uint8_t value)
{
  size_t list_count = get_list_count(list, list_size);
  for (size_t i = 0; i < list_count; i++)
  {
    if (list[i] == value)
    {
      return i;
    }
  }
  return -1;
}

bool remove_from_list(uint8_t *list, size_t list_size, uint8_t value)
{
  if (!is_in_list(list, list_size, value))
  {
    return false;
  }
  
  size_t item_index = find_in_list(list, list_size, value);
  size_t list_count = get_list_count(list, list_size);
  for (size_t i = item_index; i < list_count - 1; i++)
  {
    list[i] = list[i + 1];
  }
  
  list[list_count - 1] = 0;
  return true;
}

#endif
