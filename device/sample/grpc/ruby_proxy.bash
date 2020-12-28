#!/bin/bash

export RUBYLIB=`pwd`/lib/ruby:${RUBYLIB}

ruby $1

