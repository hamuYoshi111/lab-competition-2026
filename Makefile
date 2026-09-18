CXX      = g++
CXXFLAGS = -std=c++17 -O2 -Wall -Wextra

SOURCES  = csv_loader.cpp graph.cpp evaluate.cpp ga.cpp main.cpp
OBJECTS  = $(SOURCES:.cpp=.o)
TARGET   = orienteering

# 追加(09): 検証用テスト。本体（$(TARGET) と SOURCES）には手を入れていない。
# main.cpp を除いた共通ソースに、テストごとの main を足してビルドする。
LIB_SOURCES = csv_loader.cpp graph.cpp evaluate.cpp ga.cpp
TESTS       = local_search_test efficiency_test

$(TARGET): $(OBJECTS)
	$(CXX) $(CXXFLAGS) -o $(TARGET) $(OBJECTS)

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

local_search_test: $(LIB_SOURCES) tests/local_search_test.cpp
	$(CXX) $(CXXFLAGS) -o $@ $(LIB_SOURCES) tests/local_search_test.cpp

efficiency_test: $(LIB_SOURCES) tests/efficiency_test.cpp
	$(CXX) $(CXXFLAGS) -o $@ $(LIB_SOURCES) tests/efficiency_test.cpp

# 追加(09): 2つのテストをビルドして実行する（入力は input/ を読むので、
# このディレクトリで make test を実行すること）。
test: $(TESTS)
	./local_search_test
	./efficiency_test

clean:
	rm -f $(OBJECTS) $(TARGET) $(TESTS)

run: $(TARGET)
	./$(TARGET)

.PHONY: clean run test
