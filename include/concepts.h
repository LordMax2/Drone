#pragma once

template<typename T, typename U>
concept same_as = __is_same(T, U);
